use std::env;
use socketcan::{CANSocket, CANFrame};

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 2 {
        eprintln!("Error: Incorrect number of arguments");
        eprintln!("Usage: {} <can_id>", args[0]);
        std::process::exit(1);
    }
    println!("Arguments: {args:#?}");

    let can_id: u32 = args[1].parse().expect("Invalid CAN ID");
    send_can_message(can_id, &[0x01, 0x02, 0x03, 0x04]).expect("Failed to send CAN message");
}

fn send_can_message(can_id: u32, data: &[u8]) -> Result<(), Box<dyn std::error::Error>> {
    let socket = CANSocket::open("can0").expect("Failed to open CAN socket");
    let frame = CANFrame::new(can_id, data, false, false)?;
    socket.write_frame(&frame)?;
    println!("Sent CAN message with ID: {can_id:#X} and data: {data:?}");
    Ok(())
}
