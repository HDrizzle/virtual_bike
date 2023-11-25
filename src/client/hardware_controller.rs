// Gets raw data from the hardware on the bike and sends it to the server
// TODO: This was originally written to update to the web server, which is depreciated, must integrate with Bevy client app
use std::{error::Error, time::{Duration, Instant}, sync::{Arc, Mutex}};
use serde::{Serialize, Deserialize};
//use extras;
use serialport::SerialPortInfo;
use bevy::prelude::*;
use egui::Ui;
use bevy_renet::renet::*;
use bevy_inspector_egui::{
	bevy_egui::{egui, EguiContexts}// Importing from re-export to prevent conflicting versions of bevy_egui
};

use crate::prelude::*;

const MAX_LINE_LEN: usize = 100;// To prevent infinite loop in case hardware isn't responding correctly

// possibly useless
/*struct InputDataUpdate {
	power: f64,
	steering: f64,
	latest_update: Instant
}*/

#[derive(Serialize, Deserialize)]
pub struct Calibration {// ISO Units
	wheel_dia: f64,// Meters
	pulses_per_rev: u32,// Unitless
	power_mult: f64,// Unitless
	steering_offset: f64,// Arbitrary units, depends on hardware, applied before steering_mult
	steering_mult: f64,// Unitless, applied after steering_offset
	brake_sensitivity: f64,// Unitless
	pub timeout: u32// Seconds
}

impl Calibration {
	pub fn calibrate(&self, t_between: f64, t_last: f64, steering_raw: f64, brake_raw: f64) -> InputData {
		// Times are in microseconds
		// Steering
		let steering = (steering_raw + self.steering_offset) * self.steering_mult;
		// Power
		let t_micros = if t_between > t_last {
			t_between
		}
		else {
			t_last
		};
		let speed = (self.wheel_dia * 3.14159)/* Wheel circumference (m) */ / ((t_micros * self.pulses_per_rev as f64) / 1000000.0)/* Time per rev (S)*/;// meters / sec
		let power = speed.powf(3.0) * self.power_mult;
		// Brake
		let brake = brake_raw * self.brake_sensitivity;
		// Done
		InputData {
			steering,
			speed,
			power,
			brake
		}
	}
}

#[derive(Resource)]
pub struct HardwareInterface {// Recieves data from the bike hardware over a serial connection
	port: Arc<Mutex<Box<dyn serialport::SerialPort>>>,
	latest_update: Instant,
	cal: Calibration,
	partial_received: String
}

impl HardwareInterface {
	pub fn init(port_name: String) -> Self {
		let cal = resource_interface::load_hardware_calibration().expect("Could not load hardware calibration info");
        let port = serialport::new(&port_name, 9600)
            .timeout(Duration::from_millis(cal.timeout.into()))
            .open().expect(&format!("Failed to open port \"{}\"", &port_name));
		Self {
			port: Arc::new(Mutex::new(port)),
			latest_update: Instant::now(),
			cal,
			partial_received: String::new()
		}
	}
	pub fn get(&mut self) -> Result<InputData, Box<dyn Error>> {
        let data_string = self.readline()?;
    	self.decode(&data_string)
	}
	pub fn readline(&mut self) -> Result<String, String> {
		let mut line = String::new();
		loop {
			let mut buff: Vec<u8> = vec![0; 1];// May have to put this inside the loop
			{
				let mut port = to_string_err(self.port.lock())?;
				to_string_err(port.read(buff.as_mut_slice()))?;
			}
			let data_char = to_string_err(String::from_utf8(buff))?;
			if &data_char == "\n" {
				return Ok(line)
			}
			else {
				line.push_str(&data_char.clone());
			}
			if line.len() > MAX_LINE_LEN {
				return Err("Max line length exceeded".to_owned());
			}
		}
	}
    fn decode(&self, s: &str) -> Result<InputData, Box<dyn Error>> {
        let parts: Vec<&str> = s.split(",").collect();
		if parts.len() != 4 {return Result::Err(format!("Incorrect amount of CSVs: \"{}\"", s).into())}
		let mut n_vec = Vec::<f64>::new();
		for part in parts {
			n_vec.push(part.trim().parse::<f64>()?);
		}
		Ok(self.cal.calibrate(
			n_vec[0],
			n_vec[1],
			n_vec[2],
			n_vec[3]
		))
    }
}

fn update_system(
	mut commands: Commands,
	mut egui_contexts: EguiContexts,
	mut hardward_opt: Option<ResMut<HardwareInterface>>,
	mut renet_client: ResMut<RenetClient>,
	mut input_state: Local<String>,
	auth: Res<ClientAuth>,
) {
	match hardward_opt {
		Some(mut hardware) => {// Update latest data and send to server, TODO: eliminate code repitition
			let perf_string = match hardware.get() {
				Ok(input_data) => {
					let out = input_data.to_string();
					// send to server
					#[cfg(not(feature = "debug_vehicle_input"))]
					{
						let client_update = ClientUpdate {
							auth: auth.into_inner().clone(),
							input: input_data
						};
						renet_client.send_message(DefaultChannel::ReliableOrdered, bincode::serialize(&Request::ClientUpdate(client_update)).unwrap());// TODO: change to Unreliable
					}
					out
				},
				Err(e) => e.to_string()
			};
			egui::Window::new("Input").show(egui_contexts.ctx_mut(), |ui: &mut Ui| {
				ui.label(perf_string);
			});
		},
		None => {// If None, prompt user for serial port
			egui::Window::new("Input").show(egui_contexts.ctx_mut(), |ui: &mut Ui| {
				ui.label("Select serialport");
				let ports = serialport::available_ports().expect("No ports found!");
				for (i, p) in ports.iter().enumerate() {
					ui.label(&format!("{}. {}", i, p.port_name));
				}
				// Get input
				//let mut raw_input = String::new();
				ui.text_edit_singleline(&mut *input_state);
				// Attempt to parse input
				match input_state.trim().parse::<usize>() {
					Ok(n) => {
						if n < ports.len() {
							// Button
							if ui.button("Begin").clicked() {
								commands.insert_resource(HardwareInterface::init(ports.iter().collect::<Vec<&SerialPortInfo>>()[n].port_name.clone()));
							}
						}
					}
					Err(_) => {}// Invalid string
				}
			});
		}
	}
	// Update to egui
	//egui::Window::new("Input").show(egui_contexts.ctx_mut(), ui_callback);
}

pub struct HardwareControllerPlugin;

impl Plugin for HardwareControllerPlugin {
	fn build(&self, app: &mut App) {
		app.add_systems(Update, update_system);
		//app.insert_resource(HardwareInterface::init())// TODO
	}
}

fn select_port() -> String {
    let ports = serialport::available_ports().expect("No ports found!");
	println!("Available ports:");
    for (i, p) in ports.iter().enumerate() {
        println!("{}. {}", i, p.port_name);
    }
    let n = prompt("Port").parse::<u32>().unwrap();
    ports[n as usize].port_name.clone()
}

pub fn debug() {
	// Directly print output
	println!("Data from hardware");
    let port = select_port();
	let mut interface = HardwareInterface::init(port);
	let mut prev_print_len = 0;
	loop {
		let new_out: String = match interface.get() {
			Ok(input_data) => {
				input_data.to_string()
			},
			Err(e) => e.to_string()
		};
		// Print backspaces
		print!("{}", "\x08".repeat(prev_print_len));
		// Print new value
		print!("{}", new_out);
		prev_print_len = new_out.len();
	}
}