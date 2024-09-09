use std::env;
use socketcan::{CanDataFrame, CanFrame, CanSocket, EmbeddedFrame, Socket, StandardId};
use hex; // Import the hex module for hexadecimal decoding
use std::time::Duration;

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 4 {
        eprintln!("Error: Incorrect number of arguments");
        eprintln!("Usage: {} <can_id> <hex_message>", args[0]);
        std::process::exit(1);
    }

    let can_id: u16 = match args[1].parse() {
        Ok(id) => id,
        Err(_) => {
            eprintln!("Error: Invalid CAN ID");
            std::process::exit(1);
        }
    };

    let message = match hex::decode(&args[2]) {
        Ok(msg) => msg,
        Err(_) => {
            eprintln!("Error: Invalid hex message");
            std::process::exit(1);
        }
    };

    if message.len() > 8 {
        eprintln!("Error: Message too long. CAN messages can be at most 8 bytes.");
        std::process::exit(1);
    }

    let interface = &args[3];

    if let Err(e) = send_can_message(interface, can_id, &message) {
        eprintln!("Error: {}", e);
        std::process::exit(1);
    }

    if let Err(e) = read_can_message(interface, 5) { // 5 seconds timeout
        eprintln!("Error: {}", e);
        std::process::exit(1);
    }
}

fn send_can_message(interface: &str, can_id: u16, message: &[u8]) -> Result<(), String> {
    let socket: CanSocket = Socket::open(interface).map_err(|e| format!("Failed to open CAN socket: {}", e))?;
    let id: StandardId = StandardId::new(can_id).expect("Invalid CAN ID"); // Use StandardId::new
    let frame: CanDataFrame = CanDataFrame::new(id, message).expect("Failed to create CAN frame");
    socket.write_frame(&frame).map_err(|e| format!("Failed to send message: {}", e))?;
    println!("Message sent successfully");
    Ok(())
}

fn read_can_message(interface: &str, timeout_secs: u64) -> Result<(), String> {
    let socket: CanSocket = Socket::open(interface).map_err(|e| format!("Failed to open CAN socket: {}", e))?;
    socket.set_read_timeout(Duration::new(timeout_secs, 0)).map_err(|e| format!("Failed to set read timeout: {}", e))?;
    
    match socket.read_frame() {
        Ok(frame) => {
            println!("Message received: {:?}", frame);
            Ok(())
        },
        Err(e) => Err(format!("Failed to read message: {}", e)),
    }
}
