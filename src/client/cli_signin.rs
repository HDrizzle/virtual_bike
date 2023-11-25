/* Created 2023-11-24
Replacing the egui signin window
*/

use std::{time::Duration, thread, sync::{Arc, Mutex}};
use bevy_renet::renet::DefaultChannel;
//use extras::prompt;

use crate::{prelude::*, renet_server::{Request, Response}};
use super::{SigninEntryState, play};

fn loading_screen(go: Arc<Mutex<bool>>) {
    let chars: [&str; 4] = ["-", "\\", "|", "/"];
    let delay = Duration::from_millis(250);
    print!("-");
    let mut i: usize = 1;
    loop {
        // Print
        print!(r"\b{}", chars[i]);
        // Update i
        i += 1;
        if i >= chars.len() {
            i = 0;
        }
        // Check mutext to see of this should end
        if *go.lock().unwrap() {
            break;
        }
        // Wait
        thread::sleep(delay);
    }
}

pub fn get_play_init_info() -> Option<play::InitInfo> {
    println!("Enter information to connect to the game server");
    let name: String = prompt("Username");
    let psswd: String = prompt("Password");
    let ip: String = prompt("IPv4 addr");
    let port: String = prompt("Port");
    // Create client/transport
    #[cfg(feature = "client_default_signin")]
    let entry = SigninEntryState::default();
    #[cfg(not(feature = "client_default_signin"))]
    let entry = SigninEntryState {
        name,
        psswd,
        ip,
        port
    };
    let mut network_init_info: play::NetworkInitInfo = entry.build_network_init_info();
    let mut sent_request = false;
    // Start loading animation
    let go_mutex = Arc::new(Mutex::new(true));
    let go_mutex_clone = go_mutex.clone();
    let thread_handle = thread::spawn(move || loading_screen(go_mutex_clone));
    // Wait for server to respond
    let out: play::InitInfo = loop {
        // Renet example in README: https://crates.io/crates/renet
        let dt = Duration::from_millis(10);
        thread::sleep(dt);
        network_init_info.renet_client.update(dt);
        network_init_info.renet_transport.update(dt, &mut network_init_info.renet_client).unwrap();
        // Handle messages
        if !network_init_info.renet_client.is_disconnected() {
            // Send static data request if haven't already
            if !sent_request {
                network_init_info.renet_client.send_message(DefaultChannel::ReliableOrdered, bincode::serialize(&Request::Init).unwrap());
                sent_request = true;
            }
            // Check if static data has been recieved
            let mut static_data_opt: Option<StaticData> = None;// It won't let me `break` out of while loop so have to create this and poll it afterward
            while let Some(message) = network_init_info.renet_client.receive_message(DefaultChannel::ReliableOrdered) {
                let res = bincode::deserialize::<Response>(&message).unwrap();
                if let Response::InitState(static_data) = res {// Static data has been recieved from server, set play init info and close this window
                    println!("\nRecieved static data");
                    static_data_opt = Some(static_data);
                }
            }
            match static_data_opt {
                Some(static_data) => {
                    break play::InitInfo {
                        network: network_init_info,
                        static_data
                    };
                }
                None => {}
            }
        }
        network_init_info.renet_transport.send_packets(&mut network_init_info.renet_client);// Don't unwrap this, it will cause errors and it turns out that just ignoring them works
    };
    *go_mutex.lock().unwrap() = false;
    thread_handle.join().unwrap();
    // Done
    Some(out)
}