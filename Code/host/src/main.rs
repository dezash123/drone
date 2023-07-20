use gilrs::{Button, Event, Gilrs};
use std::io::prelude::*;
use std::net::{SocketAddr, TcpStream};
fn main() {
    //let mut gilrs = Gilrs::new().unwrap();

    //    for (_id, gamepad) in gilrs.gamepads() {
    //        println!("{} is {:?}", gamepad.name(), gamepad.power_info());
    //    }
    //    let mut active_gamepad = None;
    let addrs = [
        SocketAddr::from(([198, 168, 4, 1], 80)),
        SocketAddr::from(([127, 0, 0, 1], 80)),
    ];
    let mut stream = match TcpStream::connect(&addrs[..]) {
        Ok(s) => s,
        Err(e) => panic!("{:#?}", e),
    };
    println!("Connected to the server!");
    stream.write(&[1]).unwrap();
    stream.read(&mut [0; 128]).unwrap();
    loop {
        //        if let Some(gamepad) = active_gamepad.map(|id| gilrs.gamepad(id)) {
        //            if gamepad.is_pressed(Button::South) {
        //
        //            }
        //        }
    }
}
