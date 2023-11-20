class WheelStatic {
    
}

class VehicleStatic {

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

    constructor(obj) {// from deserialized JSON object
        this.bodyState = new BodyState(obj.body_state);
        this.input = obj.input;
        this.latestInputT = obj.latest_input_t;
    }

    update_status() {// Put's this vehicle's status in the "vehicle-status" <pre> next to the canvas
        let e = E("vehicle-info");
        if(this.input == null){e.innerHTML = "Input not available"}
        else {
            e.innerHTML = formatStringMatrix([
                ["Power", Math.round(this.input.power)],
                ["Brake", this.input.brake],
                ["Steering", this.input.steering],
                ["Latest input", formatTimeString(this.latestInputT)]
            ]);
        }
    }
}