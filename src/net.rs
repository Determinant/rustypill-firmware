use heapless::{consts::*, String, Vec, ArrayLength};
use core::fmt::Write;
use serde::{Deserialize, Serialize};
use embedded_websocket as ws;
use ws::{WebSocket, WebSocketOptions, WebSocketReceiveMessageType, WebSocketKey};
use crate::util::{GeneralError};
use crate::esp8266::{ESPLinkID, ESPWifi, WifiError, SerialTx, SerialRx, WifiError::*};

macro_rules! vec_extend {
    ($dst:expr, $s:expr) => ($dst.extend_from_slice($s).or(Err(WifiOtherError(GeneralError::BufferOverflowError))))
}

pub fn recv_http_resp<'b, T, R, U>(
        wifi: &ESPWifi<T, R>, lid: ESPLinkID,
        buff: &'b mut Vec<u8, U>) -> Result<(usize, usize), WifiError>
        where U: ArrayLength<u8>,
              T: SerialTx, R: SerialRx {
    let mut chunk = Vec::<u8, U32>::new();
    let header_off;
    let headers;
    buff.clear();
    chunk.resize(32, 0).unwrap();
    loop {
        let nread = wifi.recv(&mut chunk, lid)?;
        if nread > 0 {
            vec_extend!(buff, &chunk[..nread as usize])?;
            let mut _headers = [httparse::EMPTY_HEADER; 16];
            let mut resp = httparse::Response::new(&mut _headers);
            if let httparse::Status::Complete(off) = resp.parse(&buff).or(Err(HTTPParseError))? {
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
            let length: usize = crate::esp8266::parse_from_utf8(h.value)?;
            end += length;
        }
    }
    loop {
        let nread = wifi.recv(&mut chunk, lid)?;
        if nread > 0 {
            vec_extend!(buff,&chunk[..nread as usize])?;
        }
        if buff.len() >= end {
            break
        }
    }
    Ok((header_off, end))
}

fn gen_rest_request<T>(
        ep: &str, host: &str,
        payload: String<T>,
        buff: &mut Vec<u8, T>) -> Result<(), WifiError> 
        where T: ArrayLength<u8> {
    buff.clear();
    let mut header = String::<U128>::new();
    crate::wwrite!(header,
        "PUT {} HTTP/1.1\r\n\
         Host: {}\r\n\
         Content-type: application/json\r\n\
         Content-length: {}\r\n\r\n", ep, host, payload.len())?;
    vec_extend!(buff, header.as_bytes())?;
    vec_extend!(buff, payload.as_bytes())
}

pub fn put_rest<'b, T, R, U, V, S>(
        wifi: &ESPWifi<T, R>,
        buff: &'b mut Vec<u8, U>,
        lid: ESPLinkID,
        host: &str,
        endpoint: &str,
        req: &V) -> Result<S, WifiError> where
        T: SerialTx, R: SerialRx,
        U: ArrayLength<u8>,
        V: Serialize, S: Deserialize<'b> {
    let json: String<U> = serde_json_core::to_string(&req).or(Err(JSONGenError))?;
    gen_rest_request(endpoint, host, json, buff)?;
    wifi.send(buff, lid)?;

    let (body_off, body_end) = recv_http_resp(wifi, lid, buff)?;
    serde_json_core::from_slice(&buff[body_off..body_end]).or(Err(JSONParseError))
}

pub struct WSClient<W: rand_core::RngCore, U: ArrayLength<u8>> {
    ws: WebSocket<W>,
    ws_key: Option<WebSocketKey>,
    lid: ESPLinkID,
    buff: Vec<u8, U>
}

impl<W, U> core::fmt::Debug for WSClient<W, U> where
        W: rand_core::RngCore, U: ArrayLength<u8> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "WSClient")
    }
}

impl<W, U> WSClient<W, U> where
        W: rand_core::RngCore, U: ArrayLength<u8> {
    pub fn new(rng: W, lid: ESPLinkID, buff: Vec<u8, U>) -> Self {
        WSClient {
            ws: ws::WebSocket::new_client(rng),
            ws_key: None,
            lid,
            buff
        }
    }

    pub fn connect<T, R>(&mut self, wifi: &mut ESPWifi<T, R>, host: &str, endpoint: &str) -> Result<(), WifiError>
            where T: SerialTx, R: SerialRx {
        self.buff.resize(1024, 0).unwrap();
        let websocket_options = WebSocketOptions {
            path: endpoint,
            host: host,
            origin: "http://localhost",
            sub_protocols: None,
            additional_headers: None,
        };
        let (n, k) = self.ws.client_connect(
            &websocket_options, &mut self.buff).or(Err(WSError))?;
        self.ws_key = Some(k);
        wifi.send(&self.buff[..n], self.lid)
    }

    pub fn client_accept<T, R>(&mut self, wifi: &mut ESPWifi<T, R>) -> Result<(), WifiError>
            where T: SerialTx, R: SerialRx {
        let (_, body_end) = recv_http_resp(wifi, self.lid, &mut self.buff)?;
        match self.ws_key.as_mut() {
            Some(k) => self.ws.client_accept(&k, &self.buff[..body_end])
                              .and(Ok(())).or(Err(WSError)),
            None => Err(WSError)
        }
    }

    pub fn send_text_frame(&mut self, text: &str, buff: &mut [u8]) -> Result<usize, ()> {
        self.ws.write(ws::WebSocketSendMessageType::Text,
            true, text.as_bytes(), buff).or(Err(()))
    }
    pub fn try_recv_text_frame(&mut self, out_buff: &mut [u8]) -> Result<ws::WebSocketReadResult, ()> {
        self.ws.read(&self.buff, out_buff).or(Err(()))
    }

    pub fn recv_text_frame<T, R>(&mut self, wifi: &mut ESPWifi<T, R>, wbuff: &mut [u8]) ->
            Result<(WebSocketReceiveMessageType, usize), WifiError>
            where T: SerialTx, R: SerialRx {
        let mut chunk = Vec::<u8, U32>::new();
        self.buff.clear();
        chunk.resize(32, 0).unwrap();
        let ret;
        loop {
            let nread = wifi.recv(&mut chunk, self.lid).unwrap();
            if nread > 0 {
                vec_extend!(self.buff, &chunk[..nread as usize])?;
                match self.try_recv_text_frame(wbuff) {
                    Ok(_ret) => {
                        if _ret.end_of_message {
                            ret = _ret;
                            break
                        }
                    },
                    Err(_) => return Err(WSError)
                }
            }
        }
        chunk.clear();
        vec_extend!(chunk, &self.buff[ret.len_from..])?;
        // put the leftover
        self.buff.clear();
        vec_extend!(self.buff, &chunk)?;
        Ok((ret.message_type, ret.len_to))
    }
}

