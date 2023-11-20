// https://www.youtube.com/watch?v=GwlZ5EPu8l0
use std::env;
use virtual_bike;

fn main() {
    // Parse arguments
	let args: Vec<String> = env::args().collect();
	if args.len() < 2 {// Just the program name
		virtual_bike::client::start();
	}
    else {
        match &args[1][..] {
            "-debug-hardware" => {
				virtual_bike::client::hardware_controller::debug();
			},
			_ => panic!("Invalid arguments")
        }
    }
    
}
