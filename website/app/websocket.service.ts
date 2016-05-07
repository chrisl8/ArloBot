import {NgZone} from '@angular/core';
import {SocketService} from './SocketService.service';

export class WebSocket {
    zone:NgZone;
    webModel = {};
    socketSvc:SocketService;

    constructor() {
        this.zone = new NgZone({enableLongStackTrace: false});
        this.socketSvc = new SocketService();
        this.initListeners();
    }

    getData() {
        console.log('getData');
        this.socketSvc.emit('event');
    }

    emitValue(value) {
        console.log(value);
        this.socketSvc.emit(value);
    }
    
    sendData(value, data) {
        console.log(value, data);
        this.socketSvc.emitData(value, data);
    }

    initListeners() {
        // this.socketSvc.socket.on('success', (data) => {
        /* The ArloBot code (at the moment) emits
         * a "startup" event the first time it sees you
         * (on the initial 'connection')
         * sends the "webModel" object.
         * Every time anything changes after that it sends a
         * 'webModel" event and the same 'webModel' object.
         * 
         * In the future this may be changed since this Angular 2
         * version of the site probably doesn't need the separate
         * 'startup' event.
         */
        this.socketSvc.socket.on('startup', (data) => {
            this.zone.run(() => {
                this.webModel = data;
                console.log(this.webModel);
            });
        });
        this.socketSvc.socket.on('webModel', (data) => {
            this.zone.run(() => {
                this.webModel = data;
                console.log(this.webModel);
            });
        });
        this.socketSvc.socket.on('disconnect', (data) => {
            this.zone.run(() => {
                this.webModel = {
                    status: 'Robot offline.'
                };
                console.log('Disconnected.');
            });
        });
    }

}
