use core::fmt::Write;
use core::cell::RefCell;
use heapless::{consts::*, String};
use crate::util::GeneralError;
use crate::sync::Mutex;

pub fn parse_from_utf8<T: core::str::FromStr>(s: &[u8]) -> Result<T, WifiError> {
    core::str::from_utf8(s)
        .or(Err(SerialParseError))?.parse()
        .or(Err(SerialParseError))
}

#[macro_export]
macro_rules! wwrite {
    ($dst:expr, $($arg:tt)*) => ($crate::write!($dst, $($arg)*).or_else(|e| Err(WifiOtherError(e))))
}

pub trait SerialRx {
    fn read(&mut self) -> u8;
    fn clear(&mut self);
}

pub trait SerialTx {
    fn write(&mut self, word: u8);
    fn flush(&mut self);
}

struct ESPTx<T: SerialTx>(T);

impl<T: SerialTx> ESPTx<T> {
    pub fn new(pin: T) -> Self {
        ESPTx(pin)
    }

    pub fn write(&mut self, text: &str) {
        self.write_bytes(text.as_bytes())
    }

    pub fn write_bytes(&mut self, bytes: &[u8]) {
        for ch in bytes {
            self.0.write(*ch);
        }
        self.0.flush();
    }
}

struct ESPRx<T: SerialRx>(T);

impl<T: SerialRx> ESPRx<T> {
    pub fn new(pin: T) -> Self {
        ESPRx(pin)
    }
    pub fn jump_to_marker(&mut self, resp_marker: &str, err_marker: &str) -> Result<(), WifiError> {
        let (rm, em) = (resp_marker.as_bytes(), err_marker.as_bytes());
        let (mut i, mut rmf, mut emf) = (0, true, true);
        loop {
            let byte = self.0.read();
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
            if emf && i >= em.len() { return Err(SerialParseError) }
        }
    }

    pub fn read(&mut self, buf: &mut [u8]) {
        for b in buf.iter_mut() {
            *b = self.0.read()
        }
    }

    pub fn read_until(&mut self, marker: u8, buf: &mut [u8]) -> u16 {
        let mut i = 0;
        for b in buf.iter_mut() {
            let byte = self.0.read();
            if byte == marker { break }
            i += 1;
            *b = byte;
        }
        i
    }

    pub fn clear(&mut self) { self.0.clear() }
}

#[derive(Debug)]
pub enum WifiError {
    WifiOtherError(GeneralError),
    SerialParseError,
    HTTPParseError,
    JSONParseError,
    JSONGenError,
    WSError
}

use WifiError::*;

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

struct ESPWifiInner<T: SerialTx, R: SerialRx> {
    esp_in: RefCell<ESPTx<T>>,
    esp_out: RefCell<ESPRx<R>>,
    ssid: String<U16>,
    password: String<U32>,
}

pub struct ESPWifi<T: SerialTx, R: SerialRx> {
    inner: Mutex<ESPWifiInner<T, R>>,
    mac: [u8; 17],
}

impl<T: SerialTx, R: SerialRx> ESPWifiInner<T, R> {
    fn init(&self) -> Result<(), WifiError> {
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

    fn connect_ap(&self) -> Result<(), WifiError> {
        let mut esp_in = self.esp_in.borrow_mut();
        let mut esp_out = self.esp_out.borrow_mut();
        esp_out.clear();
        esp_in.write("AT+CWMODE=1\r\n");
        esp_out.jump_to_marker("OK", "ERROR")?;
        let mut cmd = String::<U64>::new();
        wwrite!(cmd,
                "AT+CWJAP_CUR=\"{}\",\"{}\"\r\n",
                self.ssid, self.password)?;
        esp_in.write(cmd.as_str());
        esp_out.jump_to_marker("OK", "ERROR")
    }

    fn connect(&self, ip: &str, port: u16, lid: ESPLinkID) -> Result<(), WifiError> {
        let mut esp_in = self.esp_in.borrow_mut();
        let mut esp_out = self.esp_out.borrow_mut();
        let mut cmd = String::<U64>::new();
        esp_out.clear();
        wwrite!(cmd, "AT+CIPSTART={},\"TCP\",\"{}\",{}\r\n",
                lid as u8, ip, port)?;
        esp_in.write(cmd.as_str());
        esp_out.jump_to_marker("OK", "ERROR")
    }

    fn send(&self, data: &[u8], lid: ESPLinkID) -> Result<(), WifiError> {
        let mut esp_in = self.esp_in.borrow_mut();
        let mut esp_out = self.esp_out.borrow_mut();
        let mut cmd = String::<U32>::new();
        esp_out.clear();
        for d in data.chunks(1024) {
            cmd.clear();
            wwrite!(cmd, "AT+CIPSEND={},{}\r\n",
                    lid as u8, data.len())?;
            esp_in.write(cmd.as_str());
            esp_out.jump_to_marker(">", "ERROR")?;
            esp_in.write_bytes(d);
            esp_out.jump_to_marker("SEND OK", "ERROR")?;
        }
        Ok(())
    }

    fn recv(&self, data: &mut [u8], lid: ESPLinkID) -> Result<u16, WifiError> {
        let mut esp_in = self.esp_in.borrow_mut();
        let mut esp_out = self.esp_out.borrow_mut();
        let mut cmd = String::<U32>::new();
        let mut len: [u8; 8] = [0; 8];
        let mut nread = 0;
        esp_out.clear();
        for d in data.chunks_mut(1024) {
            cmd.clear();
            wwrite!(cmd, "AT+CIPRECVDATA={},{}\r\n",
                    lid as u8, d.len())?;
            esp_in.write(cmd.as_str());
            match esp_out.jump_to_marker("+CIPRECVDATA,", "OK") {
                Ok(_) => {
                    // read length int
                    let t = esp_out.read_until(':' as u8, &mut len);
                    let clen: u16 = parse_from_utf8(&len[..t as usize])?;
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

    fn disconnect(&self, lid: ESPLinkID) -> Result<(), WifiError> {
        let mut esp_in = self.esp_in.borrow_mut();
        let mut esp_out = self.esp_out.borrow_mut();
        let mut cmd = String::<U32>::new();
        esp_out.clear();
        wwrite!(cmd, "AT+CIPCLOSE={}\r\n", lid as u8)?;
        esp_in.write(cmd.as_str());
        esp_out.jump_to_marker("OK", "ERROR")
    }

    fn status(&self, lid: ESPLinkID) -> Result<ESPConnStatus, WifiError> {
        let mut esp_in = self.esp_in.borrow_mut();
        let mut esp_out = self.esp_out.borrow_mut();
        esp_out.clear();
        esp_in.write("AT+CIPSTATUS\r\n");
        esp_out.jump_to_marker("STATUS:", "ERROR")?;
        let mut _s: [u8; 1] = [0; 1];
        esp_out.read(&mut _s);
        let mut link_status_pat = String::<U32>::new();
        wwrite!(link_status_pat, "+CIPSTATUS:{}", lid as u8)?;
        Ok(match esp_out.jump_to_marker(link_status_pat.as_str(), "OK") {
            Ok(_) => {
                esp_out.jump_to_marker("OK", "ERROR")?;
                ESPConnStatus::Connected
            },
            Err(_) => {
                let code: u8 = parse_from_utf8(&_s)?;
                if code == 5 {
                    ESPConnStatus::NoWifi
                } else {
                    ESPConnStatus::NoConn
                }
            }
        })
    }

    fn get_mac(&self, mac: &mut [u8]) -> Result<(), WifiError> {
        let mut esp_in = self.esp_in.borrow_mut();
        let mut esp_out = self.esp_out.borrow_mut();
        esp_out.clear();
        esp_in.write("AT+CIPSTAMAC_CUR?\r\n");
        esp_out.jump_to_marker("+CIPSTAMAC_CUR:\"", "ERROR")?;
        esp_out.read(mac);
        esp_out.jump_to_marker("OK", "ERROR")
    }
}


impl<T: SerialTx, R: SerialRx> ESPWifi<T, R> {
    pub fn new(esp_in: T,
               esp_out: R,
               ssid: &str, password: &str) -> Self {
        ESPWifi{
            inner: Mutex::new(ESPWifiInner {
                esp_in: RefCell::new(ESPTx::new(esp_in)),
                esp_out: RefCell::new(ESPRx::new(esp_out)),
                ssid: String::from(ssid),
                password: String::from(password)
            }),
            mac: [0; 17],
        }
    }

    pub fn init(&mut self) -> Result<(), WifiError> {
        let inner = self.inner.lock();
        inner.init()?;
        inner.get_mac(&mut self.mac)?;
        Ok(())
    }

    pub fn connect_ap(&self) -> Result<(), WifiError> {
        self.inner.lock().connect_ap()
    }

    pub fn connect(&self, ip: &str, port: u16, lid: ESPLinkID) -> Result<(), WifiError> {
        self.inner.lock().connect(ip, port, lid)
    }

    pub fn send(&self, data: &[u8], lid: ESPLinkID) -> Result<(), WifiError> {
        self.inner.lock().send(data, lid)
    }

    pub fn recv(&self, data: &mut [u8], lid: ESPLinkID) -> Result<u16, WifiError> {
        self.inner.lock().recv(data, lid)
    }

    pub fn disconnect(&self, lid: ESPLinkID) -> Result<(), WifiError> {
        self.inner.lock().disconnect(lid)
    }

    pub fn status(&self, lid: ESPLinkID) -> Result<ESPConnStatus, WifiError> {
        self.inner.lock().status(lid)
    }

    pub fn get_mac(&self) -> &str {
        core::str::from_utf8(&self.mac).unwrap()
    }
}

