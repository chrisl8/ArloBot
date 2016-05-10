// Adapted from websocket.service.ts
import {NgZone} from '@angular/core';
import {RosService} from './RosService.service';

export class RosLibJS {
    zone:NgZone;
    webModel = {}; // TODO Leftover from adaption. What do we really need?
    connected:boolean = false;
    rosLib:RosService;
    pollRosTimer:any;
    cmdVel:any;
    hostname = 'ws://' + location.hostname + ':9090';

    constructor() {
        this.zone = new NgZone({enableLongStackTrace: false});
        this.rosLib = new RosService();
        this.initListeners();
    }

    // TODO These are left over from the copy from websocket.service.ts
    // TODO What functions do we actually need?
    getData() {
        console.log('getData');
        this.rosLib.emit('event');
    }

    emitValue(value) {
        console.log(value);
        this.rosLib.emit(value);
    }

    sendData(value, data) {
        console.log(value, data);
        this.rosLib.emitData(value, data);
    }

    // TODO End of leftovers

    sendTwistCommandToROS(linear_speed:number, angular_speed:number) { // We'll use it to stop too!
        // The best way to see if this is working is:
        //rostopic echo /cmd_vel_mux/input/web # From the robot itself.
        if (this.connected && this.cmdVel !== undefined) {
            var twist = this.rosLib.twist(linear_speed, angular_speed);
            this.cmdVel.publish(twist); // cmdVel is assigned in startROSfunctions()
        } else {
            console.log("ROSLibJS NOT Connected!");
        }
    }

    // This is just a looper I made for testing.
    testTwistCommand() {
        var self = this;
        setTimeout(function () {
            // Put a closure around 'self'. Kind of like putting a bird on it.
            self.sendTwistCommandToROS(0.0, 0.0);
            self.testTwistCommand();
        }, 1000);
    }

    tryRosConnection() {
        clearTimeout(this.pollRosTimer);
        var self = this;
        this.pollRosTimer = setTimeout(function() {
            self.rosLib.socket.connect(self.hostname);
        }, 10000);
    }

    startROSfunctions() {
        console.log("Starting ROS functions");
        // These have to be set up AFTER ROS is working!

        this.cmdVel = this.rosLib.cmdVel();
        // For Testing: Just sends an infinite loop of stop messages.
        // this.testTwistCommand();
    }

    initListeners() {
        this.tryRosConnection();
        this.rosLib.socket.on('connection', () => {
            this.zone.run(() => {
                clearTimeout(this.pollRosTimer);
                this.connected = true;
                console.log('RosLibJS Connected');
                this.startROSfunctions();
            });
        });
        this.rosLib.socket.on('error', (error) => {
            this.zone.run(() => {
                this.connected = false;
                console.log('Error connecting to RosLibJS server: ', error);
            });
        });
        this.rosLib.socket.on('close', () => {
            this.zone.run(() => {
                this.connected = false;
                this.cmdVel = undefined; // I'm not sure if this is needed, but it seems smart.
                console.log('RosLibJS Disconnected.');
                this.tryRosConnection();
            });
        });
    }

}
