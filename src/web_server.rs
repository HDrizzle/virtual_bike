use std::{ops::Deref, thread, time::Duration, sync::{Arc, Mutex, mpsc}, net::{IpAddr, Ipv4Addr, SocketAddr}};
use serde_json;
//use crate::futures::Future;
//use rend3;

// for iterating enum variants https://stackoverflow.com/questions/21371534/in-rust-is-there-a-way-to-iterate-through-the-values-of-an-enum#55056427
//use strum::IntoEnumIterator;
//use strum_macros::EnumIter;

//HTTP server: https://dzone.com/articles/from-go-to-rust-with-an-http-server
use crate::hyper::rt::Future;// suggested by compiler
extern crate hyper;
extern crate futures;
use hyper::{body::Body, Request, Response, Server, Method, StatusCode, header::HeaderValue};
use hyper::service::service_fn_ok;

use crate::{ClientUpdate, resource_interface};

// Extras
use extras;

// From this crate
//use crate::vehicle::Vehicle;
use crate::world::*;

enum ServerThreadMessage {// All possible types of request
	ClientUpdate(ClientUpdate),
	NewUser {
		name: String,
		psswd: String
	}
}

pub struct WorldServer {
	world: Arc<Mutex<World>>
}

impl WorldServer {
	pub fn init(world_name: &str) -> Self {
		let world: Arc<Mutex<World>> = Arc::new(Mutex::new(World::load(world_name).expect(&format!("Failed to load world \"{}\"", world_name))));
		Self {
			world
		}
	}
	pub fn main_loop(&mut self) {
		// Start world simulation
		let map_name = self.world.lock().unwrap().map.name.clone();
		let (_world_thread_handle, world_tx, world_rx) = {
			let (tx, rx_remote) = mpsc::channel::<async_messages::ToWorld>();// To thread
			let (tx_remote, rx) = mpsc::channel::<async_messages::FromWorld>();// From thread
			let world_copy = self.world.clone();
			(thread::spawn(move || ((*world_copy).lock().unwrap()).main_loop(rx_remote, tx_remote)), tx, rx)
		};
		// Start HTTP server
		let (server_tx, server_rx) = mpsc::channel::<ServerThreadMessage>();
		let world_send_mutex: Arc<Mutex<Option<WorldSend>>> = Arc::new(Mutex::new(None));//Arc::new(Mutex::new(self.world.save()));
		let _server_thread_handle = self.spawn_server(world_send_mutex.clone(), map_name, server_tx);
		// Main loop
		loop {
			thread::sleep(Duration::from_millis(100));// Placeholder to prevent the computer from freezing
			// Receive messages from world
			loop {
				match world_rx.try_recv() {// https://doc.rust-lang.org/stable/std/sync/mpsc/struct.Receiver.html
                    Ok(update) => {
						match update {
							async_messages::FromWorld::State(world_send) => {
								// Update world send mutex
								let mut world_ref = world_send_mutex.lock().unwrap();
								*world_ref = Some(world_send);
							}
							async_messages::FromWorld::Error(e) => panic!("Received the following error from world: {}", e)
						}
                    },
                    Err(error) => {
                        match error {
                            mpsc::TryRecvError::Empty => break,
                            mpsc::TryRecvError::Disconnected => panic!("World receive channel has been disconnected")
                        }
                    }
                }
			}
			// Receive messages from server
			loop {
				match server_rx.try_recv() {// https://doc.rust-lang.org/stable/std/sync/mpsc/struct.Receiver.html
                    Ok(update) => {
						match update {
							ServerThreadMessage::ClientUpdate(update) => {
								// TODO: authenticate client
								world_tx.send(async_messages::ToWorld::ClientUpdate(update)).expect("Unable to send client update to world");
							}
							ServerThreadMessage::NewUser{name, psswd} => {
								todo!();// TODO
							}
						}
                    },
                    Err(error) => {
                        match error {
                            mpsc::TryRecvError::Empty => break,
                            mpsc::TryRecvError::Disconnected => panic!("Server receive channel has been disconnected")
                        }
                    }
                }
			}
		}
	}
    fn spawn_server(&mut self, send_mutex: Arc<Mutex<Option<WorldSend>>>, map_name: String, tx: mpsc::Sender<ServerThreadMessage>) -> thread::JoinHandle<()> {
        /*// https://docs.rs/hyper/latest/hyper/server/index.html
        let router = move || {
            let mutex_copy = send_mutex.clone();
            let tx = tx.clone();
            //let self_name_copy = self.name.clone();
            service_fn( async move |req: Request<Body>| {
				let (parts, body) = req.into_parts();
                match (parts.method, parts.uri.path(), parts.uri.query()) {
                    (Method::POST, "/client_update", _) => {
						// https://stackoverflow.com/questions/43419974/how-do-i-read-the-entire-body-of-a-tokio-based-hyper-request
						let bytes = body::to_bytes(req.into_body()).await.unwrap();
						let decoded_result: Result<ClientUpdate, serde_json::Error> = serde_json::from_str(&String::from_utf8(bytes.to_vec()).expect("response was not valid utf-8"));
						let (response, code) = match decoded_result {
							Ok(decoded) => {
								tx.send(ServerThreadMessage::ClientUpdate(decoded));
								("".to_owned(), 200)
							},
							Err(e) => (e.to_string(), 400)
						};
                        let mut res = Response::new(Body::from(response));
                        *res.status_mut() = StatusCode::from_u16(code).unwrap();
                        res
                    },
                    (Method::GET, file_path, _) => {
                        let (res_body, code) = extras::http_query_file(file_path.to_owned());
                        let mut res = Response::new(Body::from(res_body));
                        *res.status_mut() = StatusCode::from_u16(code).expect("Invalid HTTP code returned from extras::http_query_file()");
                        res
                    },
                    (_, _, _) => {
                        let mut res = Response::new(Body::from("not found"));
                        *res.status_mut() = StatusCode::NOT_FOUND;
                        res
                    }
                }
            })
        };
        let addr = get_config::get_address("virtual-bike");
        let server = Server::bind(&addr).serve(router);
		let runner = async move || {
			if let Err(e) = server.await {
				eprintln!("server error: {}", e);
			}
		};
		thread::spawn(runner)*/
		// --------------------------------------------------------------------------------------------------------------------------------------------------------
        // is single-threaded for now, TODO: thread pool
        let router = move || {
            let mutex_copy = send_mutex.clone();
            let tx = tx.clone();
			let map_name = map_name.clone();
			let send_mutex = Arc::clone(&send_mutex);
            //let self_name_copy = self.name.clone();
            service_fn_ok( move |req: Request<Body>| {
				let (parts, body) = req.into_parts();
                match (parts.method, parts.uri.path(), parts.uri.query()) {
                    (Method::POST, "/play/client_update", query_option) => {
						// https://stackoverflow.com/questions/43419974/how-do-i-read-the-entire-body-of-a-tokio-based-hyper-request
						//let bytes = body::to_bytes(req.into_body()).await.unwrap();
						let (response, code) = match query_option {
							Some(query) => {
								// replacing "%22" with """, an example of a ClientUpdate received is:
								// {%22auth%22:{%22name%22:%22admin%22,%22psswd%22:%221234%22},%22input%22:{%22steering%22:-0.0,%22power%22:1.87319796171502e-28,%22brake%22:0.0}}
								// TODO: actually fix
								let decoded_result: Result<ClientUpdate, serde_json::Error> = serde_json::from_str(&query.replace("%22", "\""));
								match decoded_result {
									Ok(decoded) => {
										tx.send(ServerThreadMessage::ClientUpdate(decoded)).unwrap();
										("".to_owned(), 200)
									},
									Err(e) => {
										println!("Bad request query to /play/client_update: \"{}\"", query);// TODO
										(e.to_string(), 400)
									}
								}
							},
							None => ("No URL Query".to_owned(), 400)
						};
                        let mut res = Response::new(Body::from(response));
                        *res.status_mut() = StatusCode::from_u16(code).unwrap();
                        res
                    },
                    (Method::GET, "/play/map.json", _) => {
						// https://stackoverflow.com/questions/43419974/how-do-i-read-the-entire-body-of-a-tokio-based-hyper-request
						//let bytes = body::to_bytes(req.into_body()).await.unwrap();
						let response = serde_json::to_string(&resource_interface::load_map(&map_name).unwrap().send()).unwrap();
                        let mut res = Response::new(Body::from(response));
                        *res.status_mut() = StatusCode::from_u16(200).unwrap();
                        res
                    },
                    (Method::GET, "/play/world.json", _) => {
						// https://stackoverflow.com/questions/43419974/how-do-i-read-the-entire-body-of-a-tokio-based-hyper-request
						//let bytes = body::to_bytes(req.into_body()).await.unwrap();
						let response: String = match send_mutex.lock().unwrap().deref() {
							Some(world_send) => serde_json::to_string(&world_send).unwrap(),
							None => "World state not available, if this is a persistant problem report to the server admin".to_owned()
						};
                        let mut res = Response::new(Body::from(response));
                        *res.status_mut() = StatusCode::from_u16(200).unwrap();
                        res
                    },
                    (Method::GET, "/play/static_vehicle", query_option) => {
						let (response, code) = match query_option {
							Some(query) => {
								match resource_interface::load_static_vehicle(query) {
									Ok(v_static) => (serde_json::to_string(&v_static).expect("Should have been able to serialize static vehicle which was deserialized"), 200),
									Err(e) => (format!("Error loading vehicle \"{}\": {}", query, e.to_string()), 500)
								}
							},
							None => ("No URL Query".to_owned(), 400)
						};
                        let mut res = Response::new(Body::from(response));
                        *res.status_mut() = StatusCode::from_u16(code).unwrap();
                        res
                    },
                    (Method::GET, file_path, _) => {
                        let (res_body, code, mime) = extras::http_query_file(file_path.to_owned());
                        let mut res = Response::new(Body::from(res_body));
						{
							let headers = res.headers_mut();
							headers.insert("Content-Type", HeaderValue::from_str(&mime).unwrap());
						  }
                        *res.status_mut() = StatusCode::from_u16(code).expect("Invalid HTTP code returned from extras::http_query_file()");
                        res
                    },
                    (_, _, _) => {
                        let mut res = Response::new(Body::from("not found"));
                        *res.status_mut() = StatusCode::NOT_FOUND;
                        res
                    }
                }
            })
        };
        let addr = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)), resource_interface::load_port().expect("Unable to load and parse port.txt"));
		println!("Starting server @ {:?}", addr);
        let server = Server::bind(&addr).serve(router);
        thread::spawn(
            || hyper::rt::run(
                server.map_err(
                    |e| {
                        eprintln!("server error: {}", e);
                    }
                )
            )
        )
    }
}