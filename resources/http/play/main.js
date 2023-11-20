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
		// Check if the static vehicles have began to load, since the static vehicle names can only be detected once the first world frame is loaded
		if(!window.staticData.vehicles.loaded && !window.staticData.vehicles.loading) {
			let staticNames = [];
			for(let [user, v_obj] of Object.entries(world_obj.vehicles)) {// https://stackoverflow.com/questions/14379274/how-to-iterate-over-a-javascript-object
				staticNames.push(v_obj.static_name);
			}
			window.staticData.beginLoadVehicles(staticNames);
		}
		// Update vehicles
		for(let [user, v_obj] of Object.entries(world_obj.vehicles)) {// https://stackoverflow.com/questions/14379274/how-to-iterate-over-a-javascript-object
			if(typeof this.vehicles[user] === 'undefined') {
				this.vehicles[user] = new Vehicle(user);
			}
			this.vehicles[user].update(v_obj);
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
		setTimeout(this.worldRequest, 1000);
	}

	updateStats() {
		E("world-fps").innerHTML = Math.round(this.worldFPS) + "";
	}

	worldRequest() {
		// Schedule next update
		if(!window.staticData.loaded){throw "worldRequest() called when static data not yet loaded";}
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

class StaticData {
	map;// Map
	vehicles;// {"staticName": VehicleStaticData}
	loadedCallback;
	loaded;// bool

	constructor(loadedCallback) {
		this.loadedCallback = loadedCallback;
		this.loaded = false;
		this.vehicles = new StaticVehicles(E("static-vehicle-load-status"));
	}

	beginLoad() {
		//asyncRequest(action, dir, sendString, contentType, doneCallback, statusElement, userMessages)
		asyncRequest(
			"GET",
			"/play/map.json",
			"",
			"",
			(status, response) => {
				this.map = new Map(JSON.parse(response));
				this.map.buildBabylonMesh(window.babylonScene);
				this.checkIfLoadComplete();
			},
			E("map-load-status"),
			["Loaded", "Error loading static map data"]
		);
	}

	beginLoadVehicles(staticNames) {
		this.vehicles.beginLoad(staticNames);
	}

	checkIfLoadComplete() {
		if(typeof this.map !== 'undefined') {
			this.loadComplete();
		}
	}

	loadComplete() {
        //alert("break (StaticData.loadComplete())");
		this.loaded = true;
		this.loadedCallback();
	}

	getVehicle(staticName) {
		if(!this.vehicles.loaded) {
			throw "Vehicles are not yet loaded";
		}
		return this.vehicles.vehicles[staticName];
	}
}

class StaticVehicles {
    vehicles;// {"staticName": VehicleStatic}
	statusElem;
	loaded;
	loading;

    constructor(statusElem) {
		this.vehicles = {};
		this.loaded = false;
		this.loading = false;
		this.statusElem = statusElem;
    }

	beginLoad(staticNames) {
		this.loading = true;
        this.statusElem.innerHTML = "Waiting...";
		for(let i = 0; i < staticNames.length; i++) {
			let name = staticNames[i];
			this.vehicles[name] = undefined;// To prevent checkIfLoadComplete() to from having false positive
			asyncRequest(
				"GET",
				"/play/static_vehicle?" + name,
				"",
				"",
				(status, response) => {
					this.vehicles[name] = new VehicleStatic(JSON.parse(response));
					this.checkIfLoadComplete();
				},
				E("hidden-element"),
				["Loaded", "Error loading static map data"]
			);
		}
	}

	checkIfLoadComplete() {
		let complete = true;
		// Go through each vehicle and check if it has been loaded
		for(let [name, v_static] of Object.entries(this.vehicles)) {// https://stackoverflow.com/questions/14379274/how-to-iterate-over-a-javascript-object
			if(typeof v_static === 'undefined') {
				complete = fale;
			}
		}
		if(complete) {
			this.loaded = true;
			this.loading = false;
			this.statusElem.innerHTML = "Loaded";
			// Can now build the vehicle meshes
			// TODO
		}
	}
}

function vectorToBabylonFormat(v) {
	return [v[0], v[1], v[2]];
}

function vectorToBabylon(vOriginal) {
	let v = vectorToBabylonFormat(vOriginal);
	return new BABYLON.Vector3(v[0], v[1], v[2]);// TODO: f(...list)
}

window.onload = function() {
	window.renderUpdater = new RenderUpdater();
	window.staticData = new StaticData(window.renderUpdater.worldRequest);
	//alert("onload break");
	window.staticData.beginLoad();
}