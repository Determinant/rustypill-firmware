#![no_main]
#![no_std]

use serde::{Deserialize, Serialize};
use core::fmt::Write;
use heapless::consts::*;
use heapless::{String, Vec};
use heapless::spsc::{Producer, Queue};

use panic_halt as _;
use cortex_m::{asm};
use stm32f1xx_hal::serial::{Serial, Config, Rx};
use stm32f1xx_hal::stm32::USART1;
use stm32f1xx_hal::prelude::*;
use embedded_hal::digital::v2::OutputPin;
use heapless::ArrayLength;

use embedded_websocket as ws;
use ws::{EmptyRng, WebSocket, WebSocketOptions, WebSocketReceiveMessageType, WebSocketKey};
//use cortex_m_semihosting::{debug, hprintln};

use rustypill::{lcd, net};
use rustypill::esp8266::{ESPLinkID, ESPConnStatus, ESPWifi, SerialTxPin, SerialRxPin, SerialTxQueue, SerialRxQueue};

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

fn check_conn<'a, T: SerialTxPin, R: SerialRxPin>(wifi: &ESPWifi<T, R>, lid: ESPLinkID, remote: &(&str, u16)) {
    match wifi.status(lid) {
        Ok(s) => match s {
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
        },
        Err(_) => panic!()
    }
}

type WifiTx = SerialTxQueue<USART1>;
type WifiRx = SerialRxQueue<'static, U2048>;

#[rtfm::app(device = stm32f1xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        rx: Rx<USART1>,
        rx_prod: Producer<'static, u8, U2048>,
        wifi: ESPWifi<WifiTx, WifiRx>,
        #[init(None)]
        ws: Option<net::WSClient<'static, EmptyRng, WifiTx, WifiRx, U2048>>,
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
            WifiTx::new(tx),
            WifiRx::new(rx_cons),
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
        match net::put_rest(wifi, &mut buff, ESPLinkID::Conn0, remote.0, &ep.as_str(), &msg) {
            Ok(resp) => {
                let resp: ServerResp = resp;
                //lcd.puts(resp.status);
            },
            Err(_) => check_conn(wifi, ESPLinkID::Conn0, &remote)
        }
        cx.spawn.test_send().unwrap();
    }

    #[task(priority = 1, resources = [wifi, lcd, ws], spawn=[test_send2_loop])]
    fn test_send2(mut cx: test_send2::Context) {
        let wifi = &mut cx.resources.wifi;
        let remote = ("192.168.1.120", 8080);
        let mut id = String::<U12>::new();
        for ch in wifi.get_mac().chars() {
            if ch != ':' { id.push(ch).unwrap() }
        }
        let mut ep = String::<U32>::new();
        *cx.resources.ws = Some(net::WSClient::new(
            EmptyRng::new(), wifi, ESPLinkID::Conn1, Vec::<u8, U2048>::new()));
        let ws = cx.resources.ws.as_ref().unwrap();
        let lid = ESPLinkID::Conn1;
        ep.write_fmt(format_args!("/{}/push", id)).unwrap();
        check_conn(wifi, lid, &remote);
        // connect WebSocket
        ws.connect(remote.0, ep.as_str()).unwrap();
        ws.client_accept().unwrap();
        // listen for text frames
        cx.spawn.test_send2_loop().unwrap();
    }

    #[task(priority = 1, resources = [wifi, lcd, ws], spawn=[test_send2_loop])]
    fn test_send2_loop(mut cx: test_send2_loop::Context) {
        let lid = ESPLinkID::Conn1;
        let lcd = &cx.resources.lcd;
        let wifi = &mut cx.resources.wifi;
        let mut buff: [u8; 1024] = [0; 1024];
        let (mt, n) = cx.resources.ws.as_ref().unwrap().recv_text_frame(&mut buff).unwrap();
        match mt {
            WebSocketReceiveMessageType::Text => lcd.puts(core::str::from_utf8(&buff[..n]).unwrap()),
            WebSocketReceiveMessageType::Ping => lcd.puts("ping!"),
            _ => asm::bkpt(),
        }
        cx.spawn.test_send2_loop().unwrap();
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            asm::wfi();
        }
    }

    #[task(binds = USART1, priority = 2, resources = [rx, rx_prod])]
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
