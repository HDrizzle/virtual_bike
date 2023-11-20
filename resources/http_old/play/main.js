class RenderUpdater {
	// Responsible for moving the camera and updating the scene

	vehicles;// {username: Vehicle}
	selectedVehicle;
	camera;
	worldFPS;

	constructor() {
		this.vehicles = {};
		this.selectedVehicle = "admin";// TODO: sign-in
		window.babylonScene = this.babylonBegin();
	}

	update(world_obj) {// MUST BE CALLED AFTER ALL STATIC DATA (MAP) IS LOADED
		// This repeatedly gets the latest simulation data and updates the scene/camera
		this.worldFPS = world_obj.fps;
		// Update vehicles
		for(let [user, v_obj] of Object.entries(world_obj.vehicles)) {// https://stackoverflow.com/questions/14379274/how-to-iterate-over-a-javascript-object
			this.vehicles[user] = new Vehicle(v_obj);
		}
		if(this.selectedVehicle != null) {
			let vSelected = this.vehicles[this.selectedVehicle];
			// Update camera orientation
			//this.camera.rotationQuaternion = BABYLON.Quaternion()
			// Position
			this.camera.target = vSelected.bodyState.translation;
			// Update status
			vSelected.update_status();
		}
		// Global stats
		this.updateStats();
		// Schedule next update
		setTimeout(this.worldRequest, 50);
	}

	updateStats() {
		E("world-fps").innerHTML = Math.round(this.worldFPS) + "";
	}

	worldRequest() {
		// Schedule next update
		let f = function(status, response){window.renderUpdater.update(JSON.parse(response))};
		asyncRequest(
            "GET",
            "/play/world.json",
            "",
            "",
            f,
            E("world-state-load-status"),
            ["Loaded", "Error loading world state"]
        );
	}
	
	createScene(engine, canvas) {
		const scene = new BABYLON.Scene(engine);

		this.camera = new BABYLON.ArcRotateCamera("camera", -Math.PI / 2, 0, 10, new BABYLON.Vector3(50, 0, 0));
		this.camera.attachControl(canvas, true);

		const light = new BABYLON.HemisphericLight("light", new BABYLON.Vector3(0, 0.5, 1));


		return scene;
	}

	babylonBegin() {// https://doc.babylonjs.com/features/introductionToFeatures/chap1/first_app
		const canvas = E("main-canvas"); // Get the canvas element
		const engine = new BABYLON.Engine(canvas, true); // Generate the BABYLON 3D engine

		// Add your code here matching the playground format
		
		const scene = this.createScene(engine, canvas); //Call the createScene function

		// Register a render loop to repeatedly render the scene
		engine.runRenderLoop(function () {
		scene.render();
		});

		// Watch for browser/canvas resize events
		window.addEventListener("resize", function () {
		engine.resize();
		});
		return scene;
	}
}

function vectorToBabylonFormat(v) {
	return [v[0], v[1], v[2]];
}

function vectorToBabylon(vOrginal) {
	let v = vectorToBabylonFormat(vOrginal);
	return new BABYLON.Vector3(v[0], v[1], v[2]);
}

window.onload = function() {
	window.renderUpdater = new RenderUpdater();
	Map.beginLoad(window.renderUpdater.worldRequest);
}