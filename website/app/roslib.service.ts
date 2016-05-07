// Adapted from websocket.service.ts
import {NgZone} from '@angular/core';
import {RosService} from './RosService.service';

export class RosLibJS {
    zone:NgZone;
    webModel = {}; // TODO Leftover from adaption. What do we really need?
    connected:boolean = false;
    rosLib:RosService;
    pollRosTimer:any;

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

    tryRosConnection() {
        clearTimeout(this.pollRosTimer);
        this.pollRosTimer = setTimeout(this.rosLib.socket.connect('ws://' + location.hostname + ':9090'), 10000);
    }

    initListeners() {
        this.tryRosConnection();
        this.rosLib.socket.on('connection', () => {
            this.zone.run(() => {
                clearTimeout(this.pollRosTimer);
                this.connected = true;
                console.log('RosLibJS Connected');
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
                console.log('RosLibJS Disconnected.');
                this.tryRosConnection();
            });
        });
    }

}
