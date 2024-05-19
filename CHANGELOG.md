# Changelog

* 2023-7-29: Major design change: Not using the web at all. Instead using bevy, and renet to send data back and forth
    The `frontend` feature is for anything only used by the client GUI and `backend` is only for the server/world simulation

* 2023-9-21: Just to clear things up, when the client connects to the game server, it is sent a bincode::serialize()ed instance of world::StaticData.
	After that only WorldSend is sent to the client as often as possible. Major change: A partial copy of the server's physics state will be sent upon initial
	connection (world::PhysicsStateSend) which means that RigidBodyHandles can be sent with body state information attached (world::BodyStates), and
	they will work with the client's rapier context (2023-12-9: All of this will be behind the `debug_render_physics` feature)

* 2023-9-23: The previous change will now be behind the `#[cfg(feature = "debug_render_physics")]` feature

* 2023-9-29: I'm bored and I have decided to implement a new feature: Chunks, like in Minecraft. This allows the game world to be gigantic,
	but not use too much memory in either the client or server.

* 2023-10-7: There is a problem. The chunk system is incompatible with sending all of the body handles and positions over, as the server's loaded chunks
	may be different from the client's. Solution: have the "debug_render_physics" feature only send the states of vehicles, wheel mounts and wheels.

* 2023-11-3: changed tachometer time units to microseconds because I am now using an optical sensor with the trainer flywheel which
	is really fast and millisecs are just not precise enough

* 2023-11-6: I have temporarily given up on world gen, so I have created the new module: crate::map::path

* 2023-11-21: Major design change for the client: due to the difficulty of having different "states" in the bevy app requiring only certain systems to be used,
	Bevy will now only be used while actually playing the game. A plain egui app using eframe will be used for the login screen.

* 2023-11-25: Started work on generic chunks, meaning that a map can be configured so that newly explored chunks will be be copied from a template and not saved,
	this will save disk space for, for example, flat or repeating terrain.

* 2023-12-2: I will attempt to upgrade to the latest version of Bevy (bevy-^0.12) as well as everything that uses it.

* 2023-12-9: I decided to not use rapier in the client except when `debug_render_physics` is enabled

* 2024-2-3: Renet doesn't handle very large pieces of data well, so I will now use a simple HTTP server to serve 3D assets and other such chunky files

* 2024-3-3: Moved this from block comment at beginning of lib.rs to CHANGELOG.md

* 2024-3-5: New feature: chat log

* 2024-3-9: Upgrading to bevy ^0.13

* 2024-5-17: I haven't been good about updating this. Anyway, Hi!