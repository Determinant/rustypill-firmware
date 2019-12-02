#![no_main]
#![no_std]

use core::cell::RefCell;

use rustypill::lcd;
use serde::{Deserialize, Serialize};
use core::fmt::Write;
use heapless::consts::*;
use heapless::{String, Vec};
use heapless::spsc::{Consumer, Producer, Queue};

use panic_halt as _;
use cortex_m::{asm};
use stm32f1xx_hal::serial::{Serial, Config, Tx, Rx};
use stm32f1xx_hal::stm32::USART1;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::stm32::Interrupt;
use embedded_hal::digital::v2::OutputPin;
use heapless::ArrayLength;
use stm32f1::stm32f103::{NVIC};

use embedded_websocket as ws;
use ws::{
    EmptyRng, WebSocket, WebSocketOptions, WebSocketReceiveMessageType, WebSocketSendMessageType,
    WebSocketServer, WebSocketState, WebSocketKey
};
use cortex_m_semihosting::{debug, hprintln};

pub struct SerialRx<'a, T: ArrayLength<u8>> {
    rx_cons: Consumer<'a, u8, T>
}

impl<'a, T: ArrayLength<u8>> SerialRx<'a, T> {
    pub fn new(rx_cons: Consumer<'a, u8, T>) -> Self {
        SerialRx{ rx_cons }
    }
    pub fn jump_to_marker(&mut self, resp_marker: &str, err_marker: &str) -> Result<(), ()> {
        let (rm, em) = (resp_marker.as_bytes(), err_marker.as_bytes());
        let (mut i, mut rmf, mut emf) = (0, true, true);
        loop {
            if let Some(byte) = self.rx_cons.dequeue() {
                if byte == '\r' as u8 || byte == '\n' as u8 {
                    i = 0;
                    rmf = true;
                    emf = true;
                    continue
                }
                if rmf && byte != rm[i] {
                    rmf = false;
                }
                if emf && byte != em[i] {
                    emf = false;
                }
                i += 1;
                if rmf && i >= rm.len() { return Ok(()) }
                if emf && i >= em.len() { return Err(()) }
            }
        }
    }

    pub fn read(&mut self, buf: &mut [u8]) {
        for b in buf.iter_mut() {
            while !self.rx_cons.ready() {}
            *b = self.rx_cons.dequeue().unwrap();
        }
    }

    pub fn read_until(&mut self, marker: u8, buf: &mut [u8]) -> u16 {
        let mut i = 0;
        for b in buf.iter_mut() {
            while !self.rx_cons.ready() {}
            let byte = self.rx_cons.dequeue().unwrap();
            if byte == marker { break }
            i += 1;
            *b = byte;
        }
        i
    }

    pub fn clear(&mut self) {
        while let Some(_) = self.rx_cons.dequeue() {}
    }
}

pub struct SerialTx {
    tx: Tx<USART1>
}

impl SerialTx {
    pub fn new(tx: Tx<USART1>) -> Self {
        SerialTx { tx }
    }

    pub fn write(&mut self, text: &str) {
        for ch in text.as_bytes() {
            nb::block!(self.tx.write(*ch)).unwrap();
        }
        nb::block!(self.tx.flush()).unwrap();
    }

    pub fn write_bytes(&mut self, bytes: &[u8]) {
        for ch in bytes {
            nb::block!(self.tx.write(*ch)).unwrap();
        }
        nb::block!(self.tx.flush()).unwrap();
    }

}

struct MutexGuard<T>(*mut T, bool);

impl<T> Drop for MutexGuard<T> {
    fn drop(&mut self) {
        if self.1 {
            unsafe { NVIC::unmask(Interrupt::USART2); }
        }
    }
}

impl<T> core::ops::Deref for MutexGuard<T> {
    type Target = T;
    fn deref(&self) -> &T { unsafe {&*self.0} }
}

impl<T> core::ops::DerefMut for MutexGuard<T> {
    fn deref_mut(&mut self) -> &mut T { unsafe {&mut *self.0} }
}

struct Mutex<T> {
    data: core::cell::UnsafeCell<T>
}

impl<T> Mutex<T> {
    pub fn new(v: T) -> Self {
        Mutex {
            data: core::cell::UnsafeCell::new(v)
        }
    }

    pub fn lock(&self) -> MutexGuard<T> {
        let unmask = if NVIC::is_enabled(Interrupt::USART2) {
            NVIC::mask(Interrupt::USART2);
            true
        } else { false };
        MutexGuard(self.data.get(), unmask)
    }
}

unsafe impl<T> Sync for Mutex<T> {}

#[derive(Copy, Clone)]
pub enum ESPLinkID {
    Conn0 = 0,
    Conn1 = 1,
    Conn2 = 2,
    Conn3 = 3,
    Conn4 = 4
}

pub enum ESPConnStatus {
    NoWifi,
    NoConn,
    Connected
}

struct ESPWifiInner<'a, T: ArrayLength<u8>> {
    esp_in: RefCell<SerialTx>,
    esp_out: RefCell<SerialRx<'a, T>>,
    ssid: String<U16>,
    password: String<U32>,
}

pub struct ESPWifi<'a, T: ArrayLength<u8>> {
    inner: Mutex<ESPWifiInner<'a, T>>,
    mac: [u8; 17],
}

fn gen_json_rest<T: ArrayLength<u8>>(ep: &str, host: &str, json: &str, rest: &mut String<T>) {
    rest.write_fmt(format_args!(
        "PUT {} HTTP/1.1\r\n\
         Host: {}\r\n\
         Content-type: application/json\r\n\
         Content-length: {}\r\n\r\n\
         {}", ep, host, json.len(), json)).unwrap()
}

impl<'a,  T: ArrayLength<u8>> ESPWifiInner<'a, T> {
    fn init(&self) -> Result<(), ()> {
        let mut esp_in = self.esp_in.borrow_mut();
        let mut esp_out = self.esp_out.borrow_mut();
        esp_in.write("ATE0\r\n");
        esp_out.jump_to_marker("OK", "ERROR")?;
        esp_in.write("AT+CIPRECVMODE=1\r\n");
        esp_out.jump_to_marker("OK", "ERROR")?;
        esp_in.write("AT+CIPMUX=1\r\n");
        esp_out.jump_to_marker("OK", "ERROR")?;
        Ok(())
    }

    fn connect_ap(&self) -> Result<(), ()> {
        let mut esp_in = self.esp_in.borrow_mut();
        let mut esp_out = self.esp_out.borrow_mut();
        esp_in.write("AT+CWMODE=1\r\n");
        esp_out.jump_to_marker("OK", "ERROR")?;
        let mut cmd = String::<U128>::new();
        cmd.write_fmt(format_args!("AT+CWJAP_CUR=\"{}\",\"{}\"\r\n", self.ssid, self.password)).unwrap();
        esp_in.write(cmd.as_str());
        esp_out.jump_to_marker("OK", "ERROR")
    }

    fn connect(&self, ip: &str, port: u16, lid: ESPLinkID) -> Result<(), ()> {
        let mut esp_in = self.esp_in.borrow_mut();
        let mut esp_out = self.esp_out.borrow_mut();
        let mut cmd = String::<U64>::new();
        cmd.write_fmt(format_args!("AT+CIPSTART={},\"TCP\",\"{}\",{}\r\n", lid as u8, ip, port)).unwrap();
        esp_in.write(cmd.as_str());
        esp_out.jump_to_marker("OK", "ERROR")
    }

    fn send(&self, data: &[u8], lid: ESPLinkID) -> Result<(), ()> {
        let mut esp_in = self.esp_in.borrow_mut();
        let mut esp_out = self.esp_out.borrow_mut();
        let mut cmd = String::<U32>::new();
        for d in data.chunks(1024) {
            cmd.clear();
            cmd.write_fmt(format_args!("AT+CIPSEND={},{}\r\n", lid as u8, data.len())).unwrap();
            esp_in.write(cmd.as_str());
            esp_out.jump_to_marker("OK", "ERROR")?;
            esp_out.jump_to_marker(">", "ERROR")?;
            esp_in.write_bytes(d);
            esp_out.jump_to_marker("SEND OK", "ERROR")?;
        }
        Ok(())
    }

    fn recv(&self, data: &mut [u8], lid: ESPLinkID) -> Result<u16, ()> {
        let mut esp_in = self.esp_in.borrow_mut();
        let mut esp_out = self.esp_out.borrow_mut();
        let mut cmd = String::<U32>::new();
        let mut len: [u8; 8] = [0; 8];
        let mut nread = 0;
        esp_out.clear();
        for d in data.chunks_mut(1024) {
            cmd.clear();
            cmd.write_fmt(format_args!("AT+CIPRECVDATA={},{}\r\n", lid as u8, d.len())).unwrap();
            esp_in.write(cmd.as_str());
            match esp_out.jump_to_marker("+CIPRECVDATA,", "OK") {
                Ok(_) => {
                    // read length int
                    let t = esp_out.read_until(':' as u8, &mut len);
                    let clen: u16 = core::str::from_utf8(&len[..t as usize]).unwrap().parse().unwrap();
                    // read the chunk of data
                    esp_out.read(&mut d[..clen as usize]);
                    nread += clen;
                    if (clen as usize) < d.len() {
                        break
                    }
                    esp_out.jump_to_marker("OK", "ERROR")?;
                },
                Err(_) => {
                    break
                }
            }
        }
        Ok(nread)
    }

    fn disconnect(&self, lid: ESPLinkID) -> Result<(), ()> {
        let mut esp_in = self.esp_in.borrow_mut();
        let mut esp_out = self.esp_out.borrow_mut();
        let mut cmd = String::<U32>::new();
        cmd.write_fmt(format_args!("AT+CIPCLOSE={}\r\n", lid as u8)).unwrap();
        esp_in.write(cmd.as_str());
        esp_out.jump_to_marker("OK", "ERROR")
    }

    fn status(&self, lid: ESPLinkID) -> ESPConnStatus {
        let mut esp_in = self.esp_in.borrow_mut();
        let mut esp_out = self.esp_out.borrow_mut();
        esp_in.write("AT+CIPSTATUS\r\n");
        esp_out.jump_to_marker("STATUS:", "ERROR").unwrap();
        let mut _s: [u8; 1] = [0; 1];
        esp_out.read(&mut _s);
        let mut link_status_pat = String::<U32>::new();
        link_status_pat.write_fmt(format_args!("+CIPSTATUS:{}", lid as u8)).unwrap();
        match esp_out.jump_to_marker(link_status_pat.as_str(), "OK") {
            Ok(_) => {
                esp_out.jump_to_marker("OK", "ERROR").unwrap();
                ESPConnStatus::Connected
            },
            Err(_) => {
                let code: u8 = core::str::from_utf8(&_s).unwrap().parse().unwrap();
                if code == 5 {
                    ESPConnStatus::NoWifi
                } else {
                    ESPConnStatus::NoConn
                }
            }
        }
    }

    fn get_mac(&self, mac: &mut [u8]) {
        let mut esp_in = self.esp_in.borrow_mut();
        let mut esp_out = self.esp_out.borrow_mut();
        esp_in.write("AT+CIPSTAMAC_CUR?\r\n");
        esp_out.jump_to_marker("+CIPSTAMAC_CUR:\"", "ERROR").unwrap();
        esp_out.read(mac);
        esp_out.jump_to_marker("OK", "ERROR").unwrap();
    }
}


impl<'a,  T: ArrayLength<u8>> ESPWifi<'a, T> {
    pub fn new(esp_in: SerialTx,
               esp_out: SerialRx<'a, T>,
               ssid: &str, password: &str) -> Self {
        ESPWifi{
            inner: Mutex::new(ESPWifiInner {
                esp_in: RefCell::new(esp_in),
                esp_out: RefCell::new(esp_out),
                ssid: String::from(ssid),
                password: String::from(password)
            }),
            mac: [0; 17],
        }
    }

    pub fn init(&mut self) -> Result<(), ()> {
        let inner = self.inner.lock();
        inner.init()?;
        inner.get_mac(&mut self.mac);
        Ok(())
    }

    pub fn connect_ap(&self) -> Result<(), ()> {
        self.inner.lock().connect_ap()
    }

    pub fn connect(&self, ip: &str, port: u16, lid: ESPLinkID) -> Result<(), ()> {
        self.inner.lock().connect(ip, port, lid)
    }

    pub fn send(&self, data: &[u8], lid: ESPLinkID) -> Result<(), ()> {
        self.inner.lock().send(data, lid)
    }

    pub fn recv(&self, data: &mut [u8], lid: ESPLinkID) -> Result<u16, ()> {
        self.inner.lock().recv(data, lid)
    }

    pub fn disconnect(&self, lid: ESPLinkID) -> Result<(), ()> {
        self.inner.lock().disconnect(lid)
    }

    pub fn status(&self, lid: ESPLinkID) -> ESPConnStatus {
        self.inner.lock().status(lid)
    }

    pub fn get_mac(&self) -> &str {
        core::str::from_utf8(&self.mac).unwrap()
    }
}

#[derive(Serialize, Deserialize)]
struct TestMsg<'a> {
    cmd: &'a str,
    mac: &'a str,
    temp: u32,
    timestamp: u32,
}

#[derive(Serialize, Deserialize)]
struct ServerResp<'a> {
    status: &'a str
}

fn check_conn<'a, T: ArrayLength<u8>>(wifi: &ESPWifi<'a, T>, lid: ESPLinkID, remote: &(&str, u16)) {
    match wifi.status(lid) {
        ESPConnStatus::NoWifi => {
            while let Err(_) = wifi.connect_ap() {
                cortex_m::asm::delay(40000000);
            }
            while let Err(_) = wifi.connect(remote.0, remote.1, lid) {
                cortex_m::asm::delay(40000000);
            }
        },
        ESPConnStatus::NoConn => {
            while let Err(_) = wifi.connect(remote.0, remote.1, lid) {
                cortex_m::asm::delay(40000000);
            }
        }
        _ => return
    }
}

fn recv_resp<'a, 'b, T: ArrayLength<u8>, U: ArrayLength<u8>>(
        wifi: &ESPWifi<'a, T>, lid: ESPLinkID, buff: &'b mut Vec<u8, U>) -> (usize, usize) {
    let mut chunk = Vec::<u8, U32>::new();
    let header_off;
    let headers;
    chunk.resize(32, 0).unwrap();
    loop {
        let nread = wifi.recv(&mut chunk, lid).unwrap();
        if nread > 0 {
            buff.extend_from_slice(&chunk[..nread as usize]).unwrap();
            let mut _headers = [httparse::EMPTY_HEADER; 16];
            let mut resp = httparse::Response::new(&mut _headers);
            if let httparse::Status::Complete(off) = resp.parse(&buff).unwrap() {
                header_off = off;
                headers = _headers;
                break
            }
        }
    }
    const LENGTH_NAME: &str = "Content-Length";
    let mut end = header_off;
    for h in headers.iter() {
        if h.name.len() != LENGTH_NAME.len() { continue }
        let mut flag = true;
        for (a, b) in h.name.as_bytes().iter().zip(LENGTH_NAME.as_bytes()) {
            let a = *a;
            let b = *b;
            let a = if a < 97 { a + 32 } else { a };
            let b = if b < 97 { b + 32 } else { b };
            if a != b {
                flag = false;
                break
            }
        }
        if flag { 
            let length: usize = core::str::from_utf8(h.value).unwrap().parse().unwrap();
            end += length;
        }
    }
    loop {
        let nread = wifi.recv(&mut chunk, lid).unwrap();
        if nread > 0 {
            buff.extend_from_slice(&chunk[..nread as usize]).unwrap();
        }
        if buff.len() >= end {
            break
        }
    }
    (header_off, end)
}

fn rest_put<'a, 'b, T: ArrayLength<u8>, U: ArrayLength<u8>, R: Serialize, S: Deserialize<'b>>(
        wifi: &ESPWifi<'a, T>, _lcd: &lcd::LCD1602, buff: &'b mut Vec<u8, U>,
        lid: ESPLinkID,
        host: &str,
        endpoint: &str,
        req: &R) -> Result<S, ()> {
    {
        let mut rest = String::<U2048>::new();
        let json: String<U2048> = serde_json_core::to_string(&req).unwrap();
        gen_json_rest(endpoint, host, json.as_str(), &mut rest);
        wifi.send(rest.as_str().as_bytes(), lid)?;
    }
    {
        let (body_off, body_end) = recv_resp(wifi, lid, buff);
        let resp_json = core::str::from_utf8(&buff[body_off..body_end]).unwrap();
        Ok(serde_json_core::from_str(resp_json).unwrap())
    }
}

struct WSClient<T: rand_core::RngCore> {
    ws: WebSocket<T>,
    ws_key: Option<WebSocketKey>
}

impl<T: rand_core::RngCore> WSClient<T> {
    fn new(rng: T) -> Self {
        WSClient {
            ws: ws::WebSocket::new_client(rng),
            ws_key: None
        }
    }

    fn connect(&mut self, host: &str, endpoint: &str, buff: &mut [u8]) -> Result<usize, ()> {
        let websocket_options = WebSocketOptions {
            path: endpoint,
            host: host,
            origin: "http://localhost",
            sub_protocols: None,
            additional_headers: None,
        };
        let (n, k) = self.ws.client_connect(&websocket_options, buff).or(Err(()))?;
        self.ws_key = Some(k);
        Ok(n)
    }

    fn client_accept(&mut self, buff: &[u8]) -> Result<(), ()> {
        match self.ws_key.as_mut() {
            Some(k) => self.ws.client_accept(&k, buff).and(Ok(())).or(Err(())),
            None => Err(())
        }
    }

    fn send_text_frame(&mut self, text: &str, buff: &mut [u8]) -> Result<usize, ()> {
        self.ws.write(ws::WebSocketSendMessageType::Text,
            true, text.as_bytes(), buff).or(Err(()))
    }
    fn recv_text_frame(&mut self, in_buff: &[u8], out_buff: &mut [u8]) -> Result<ws::WebSocketReadResult, ()> {
        self.ws.read(in_buff, out_buff).or(Err(()))
    }
}

#[rtfm::app(device = stm32f1xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        rx: Rx<USART1>,
        rx_prod: Producer<'static, u8, U2048>,
        wifi: ESPWifi<'static, U2048>,
        lcd: lcd::LCD1602
    }

    #[init(spawn = [test])]
    fn init(cx: init::Context) -> init::LateResources {
        static mut RX_RB: Option<Queue<u8, U2048>> = None;
        *RX_RB = Some(Queue::new());

        let (rx_prod, rx_cons) = RX_RB.as_mut().unwrap().split();

        let p = cx.device;
        let mut rcc = p.RCC.constrain();
        let mut afio = p.AFIO.constrain(&mut rcc.apb2);

        //let cp = cx.core;
        let mut flash = p.FLASH.constrain();
        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = p.GPIOB.split(&mut rcc.apb2);
        let mut rst = gpiob.pb7.into_open_drain_output(&mut gpiob.crl);
        // reset the WiFi module and wait for boot up
        //cortex_m::asm::delay(4000000);
        rst.set_high().unwrap();
        cortex_m::asm::delay(4000000);
        let lcd = lcd::LCD1602::new();
        lcd.init(lcd::LCD16X2_DISPLAY_ON_CURSOR_OFF_BLINK_OFF);
        lcd.set_backlight(true);

        let pin_tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let pin_rx = gpioa.pa10; //.into_floating_input(&mut gpioa.crh);
        let mut serial = Serial::usart1(
            p.USART1,
            (pin_tx, pin_rx),
            &mut afio.mapr,
            Config::default()
            .baudrate(115200.bps())
            .stopbits(stm32f1xx_hal::serial::StopBits::STOP1),
            clocks,
            &mut rcc.apb2,
        );

        serial.listen(stm32f1xx_hal::serial::Event::Rxne);
        //serial.listen(stm32f1xx_hal::serial::Event::Txe);
        let (tx, rx) = serial.split();
        let wifi = ESPWifi::new(
            SerialTx::new(tx),
            SerialRx::new(rx_cons),
            "R1F",
            "worldhello"
        );
        cx.spawn.test().unwrap();
        init::LateResources {
            wifi,
            rx,
            rx_prod,
            lcd
        }
    }

    #[task(priority = 1, resources = [wifi, lcd], spawn = [test_send, test_send2])]
    fn test(cx: test::Context) {
        cx.resources.wifi.init().unwrap();
        cx.spawn.test_send().unwrap();
        cx.spawn.test_send2().unwrap();
    }

    #[task(priority = 1, resources = [wifi, lcd], spawn=[test_send])]
    fn test_send(mut cx: test_send::Context) {
        let wifi = &mut cx.resources.wifi;
        let lcd = &cx.resources.lcd;
        let remote = ("192.168.1.120", 8080);
        let mut id = String::<U12>::new();
        for ch in wifi.get_mac().chars() {
            if ch != ':' { id.push(ch).unwrap() }
        }
        let mut ep = String::<U32>::new();
        let mut buff = Vec::<u8, U2048>::new();
        ep.write_fmt(format_args!("/{}/update", id)).unwrap();
        check_conn(wifi, ESPLinkID::Conn0, &remote);
        let msg = TestMsg {
            cmd: "hi",
            mac: wifi.get_mac(),
            temp: 24,
            timestamp: 0xffff
        };
        cortex_m::asm::delay(8000000);
        match rest_put(wifi, lcd, &mut buff, ESPLinkID::Conn0, remote.0, &ep.as_str(), &msg) {
            Ok(resp) => {
                let resp: ServerResp = resp;
                lcd.puts(resp.status);
            },
            Err(_) => check_conn(wifi, ESPLinkID::Conn0, &remote)
        }
        cx.spawn.test_send().unwrap();
    }

    #[task(priority = 1, resources = [wifi, lcd], spawn=[test_send2])]
    fn test_send2(mut cx: test_send2::Context) {
        let wifi = &mut cx.resources.wifi;
        let lcd = &cx.resources.lcd;
        let remote = ("192.168.1.120", 8080);
        let mut id = String::<U12>::new();
        for ch in wifi.get_mac().chars() {
            if ch != ':' { id.push(ch).unwrap() }
        }
        let mut ep = String::<U32>::new();
        //let mut ep2 = String::<U32>::new();
        //let mut ws_cli = WSClient::new(EmptyRng::new());
        let mut buff = Vec::<u8, U2048>::new();
        ep.write_fmt(format_args!("/{}/update", id)).unwrap();
        //ep2.write_fmt(format_args!("/{}/push", id)).unwrap();
        check_conn(wifi, ESPLinkID::Conn1, &remote);
        //buff.resize(1024, 0).unwrap();
        //let nbytes = ws_cli.connect(remote.0, ep2.as_str(), &mut buff).unwrap();
        //wifi.send(&buff[..nbytes], ESPLinkID::Conn1).unwrap();
        let msg = TestMsg {
            cmd: "hello",
            mac: wifi.get_mac(),
            temp: 25,
            timestamp: 0xffff
        };
        cortex_m::asm::delay(8000000);
        match rest_put(wifi, lcd, &mut buff, ESPLinkID::Conn1, remote.0, &ep.as_str(), &msg) {
            Ok(resp) => {
                let resp: ServerResp = resp;
                //lcd.puts(resp.status);
            },
            Err(_) => check_conn(wifi, ESPLinkID::Conn1, &remote)
        }
        cx.spawn.test_send2().unwrap();
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            asm::wfi();
        }
    }

    #[task(binds = USART1, priority = 3, resources = [rx, rx_prod])]
    fn usart1_rx(cx: usart1_rx::Context) {
        let rx = cx.resources.rx;
        loop {
            match rx.read() {
                Ok(byte) => {
                    match cx.resources.rx_prod.enqueue(byte) {
                        Ok(_) => {
                        }
                        Err(_) => asm::bkpt(),
                    }
                }
                Err(nb::Error::Other(stm32f1xx_hal::serial::Error::Overrun)) =>
                    asm::bkpt(),
                _ => break
            }
        }
    }

    extern "C" {
        fn USART2();
    }
};
