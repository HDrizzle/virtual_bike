class MapElevMesh {
    // Represents world::ReqularElevationMeshSend
    precision;
    vertices;
    indices;

    constructor(obj) {
        this.precision = obj.precision;
        this.vertices = obj.vertices;
        this.indices = obj.indices;
    }

    buildBabylonMesh(scene) {
        // https://doc.babylonjs.com/features/featuresDeepDive/mesh/creation/custom/custom
        var customMesh = new BABYLON.Mesh("map", scene);

        // example
        /*var positions = [-5, 2, -3, -7, -2, -3, -3, -2, -3, 5, 2, 3, 7, -2, 3, 3, -2, 3];
        var indices = [0, 1, 2, 3, 4, 5];*/

        var positions = [];
        var indices = [];

        // Flatten vertices
        for(var i = 0; i < this.vertices.length; i++) {
            var v = this.vertices[i];
            positions = positions.concat(vectorToBabylonFormat(v));
        }

        // Flatten indices
        for(var i = 0; i < this.indices.length; i++) {
            indices = indices.concat(this.indices[i]);
        }
            
        var vertexData = new BABYLON.VertexData();

        vertexData.positions = positions;
        vertexData.indices = indices;	

        vertexData.applyToMesh(customMesh);
    }
}

class Map {
    // STATIC, is only loaded once
    // Represents map data loaded from the server by serializing world::Map
    elevMesh;
    size;
    landmarks;
    bgColor;

    constructor(obj) {
        this.elevMesh = new MapElevMesh(obj.elevation);
        this.size = obj.size;
        this.landmarks = obj.landmarks;
        this.bgColor = obj.background_color;
        // Static vehicle data
        /*this.vehicles = {};
        for(let [user, v_obj] of Object.entries(world_obj.vehicles)) {
            this.vehicles[user] = new VehicleStaticData(v_obj);
        }*/
    }

    buildBabylonMesh(scene) {
        //alert("Map.buildBabylonMesh()");
        this.elevMesh.buildBabylonMesh(scene);
    }
    /*
    static beginLoad(done_callback) {
        //asyncRequest(action, dir, sendString, contentType, doneCallback, statusElement, userMessages)
        asyncRequest(
            "GET",
            "/play/map.json",
            "",
            "",
            function(status, response){Map.loadCallback(status, response, done_callback)},
            E("map-load-status"),
            ["Loaded", "Error loading static map data"]
        )
    }
    static loadCallback(status, response, done_callback) {
        window.map = new Map(JSON.parse(response));
        window.map.buildBabylonMesh(window.babylonScene);
        done_callback();
    }*/
}