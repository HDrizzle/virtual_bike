class WheelStatic {
    
}

class VehicleStatic {
    obj;
    mesh;

    constructor(obj) {
        this.obj = obj;
    }

    getMeshData() {// -> BABYLON.VertexData
        // https://doc.babylonjs.com/features/featuresDeepDive/mesh/creation/custom/custom
        // example of moving a mesh https://www.babylonjs-playground.com/#KQX0WU

        // TODO get mesh data from WASM
        var vertices = [];
        var indices = [];

        var positionsFlattened = [];
        var indicesFlattened = [];

        // Flatten vertices
        for(var i = 0; i < vertices.length; i++) {
            var v = this.vertices[i];
            positionsFlattened = positionsFlattened.concat(vectorToBabylonFormat(v));
        }

        // Flatten indices
        for(var i = 0; i < indices.length; i++) {
            indicesFlattened = indicesFlattened.concat(this.indices[i]);
        }
            
        var vertexData = new BABYLON.VertexData();

        vertexData.positions = positionsFlattened;
        vertexData.indices = indicesFlattened;	

        return vertexData;
    }
}

class BodyState {
    translation;
    rotation;
    linVel;

    constructor(obj) {// from deserialized JSON object
        var l = obj.translation;
        this.translation = vectorToBabylon(obj.translation);
        this.rotation = obj.rotation;
        this.linVel = obj.velocity;
    }
}

class Vehicle {
    // Resembles vehicle::VehicleSend in the Rust source
    bodyState;
    input;
    staticName;
    mesh;
    name;

    constructor(name) {
        this.name = name;
    }
    
    createBabylonMesh(scene) {// -> BABYLON.Mesh
        // https://doc.babylonjs.com/features/featuresDeepDive/mesh/creation/custom/custom
        // example of moving a mesh https://www.babylonjs-playground.com/#KQX0WU
        var mesh = new BABYLON.Mesh("vehicle", scene);
        vertexData.applyToMesh(window.staticData.getVehicle(this.name).getMeshData());
        this.mesh = mesh;
    }

    update(obj) {// from deserialized JSON object
        this.bodyState = new BodyState(obj.body_state);
        this.input = obj.input;
        this.latestInputT = obj.latest_input_t;
        this.staticName = obj.static_name;
    }

    update_status() {// Put's this vehicle's status in the "vehicle-status" <pre> next to the canvas
        //alert("Vehicle update status break");
        let e = E("vehicle-info");
        let coords = this.bodyState.translation;
        let status = "";
        if(this.input == null){status += "Input not available"}
        else {
            status += formatStringMatrix([
                ["Power", Math.round(this.input.power)],
                ["Brake", this.input.brake],
                ["Steering", this.input.steering],
                ["Latest input", formatTimeString(this.latestInputT)]
            ]);
        }
        status += `\nX: ${Math.round(coords.x)}, Y: ${Math.round(coords.y)}, Z: ${Math.round(coords.z)}`;
        e.innerHTML = status;
    }
}