import {Component} from '@angular/core';
import {BoolToYesNo} from './booleanToYesNo.pipe';
import {BoolToOnOff} from './boolToOnOff.pipe';
import {CORE_DIRECTIVES, FORM_DIRECTIVES} from '@angular/common';
// http://stackoverflow.com/a/34546950/4982408
import {ACCORDION_DIRECTIVES} from 'ng2-bootstrap/ng2-bootstrap';

import {WebSocket} from './websocket.service';
import {RosLibJS} from './roslib.service';

// Fake out TypeScript regarding a generic JS librarys
declare var ROSLIB:any;
declare var VirtualJoystick:any;

@Component({
    selector: 'my-app',
// http://stackoverflow.com/questions/30580083/angular2-no-provider-for-nameservice
    providers: [WebSocket, RosLibJS],
    directives: [ACCORDION_DIRECTIVES, CORE_DIRECTIVES, FORM_DIRECTIVES],
    templateUrl: 'arlobot.html',
    pipes: [BoolToYesNo, BoolToOnOff]
    /* Good stuff here:
     *http://www.syntaxsuccess.com/viewarticle/input-controls-in-angular-2.0
     *
     * Also, get on with the tutorial!
     */
})

export class AppComponent {
    arlobotSvc:WebSocket;
    rosSvc:RosLibJS;
    data:number = 1;

// For Accordian demo:
// http://valor-software.com/ng2-bootstrap/
    public oneAtATime:boolean = false;
    public items:Array<string> = ['Item 1', 'Item 2', 'Item 3'];
    public joystickOutput:string = 'Off';
    public newMapName:string = '';
    public newWaypointName:string = '';
    public newThingToSay:string = '';

    goToLogStreamer() {
        window.open('http://' + location.hostname + ':28778/');
        // window.location.href='http://www.cnn.com/';
    }

    public status:Object = {
        isFirstOpen: true,
        isFirstDisabled: false,
        openStartup: true
    };

    public groups:Array<any> = [
        {
            title: 'Dynamic Group Header - 1',
            content: 'Dynamic Group Body - 1'
        },
        {
            title: 'Dynamic Group Header - 2',
            content: 'Dynamic Group Body - 2'
        }
    ];

    public addItem():void {
        this.items.push(`Items ${this.items.length + 1}`);
    }

    // END Accordian demo bits.

    constructor(mySvc:WebSocket, rosLibSvc:RosLibJS) {
        this.arlobotSvc = mySvc;
        this.rosSvc = rosLibSvc;
    }

    /*
     webModel is sent to us by the robot.
     We never update it.
     NOICE: we use [ngModel] NOT [(ngModel)]!!!
     We only send events TO the robot,
     the robot updates the webModel as it sees fit,
     and returns an updated webModel to us IF it wants to.
     The only down side is this means that switches can
     sometimes be tricky. We may "switch" them,
     and then they go back when the webModel updates,
     if the change didn't work.
     HOWEVER, this is good, it visually shows us that
     the robot refused the input.
     Note this could also all just be crappy programming. :)
     */

    public updateWebModelKey(key, valueIfAlreadyTrue, valueIfAlreadyFalse):void {
        var send = valueIfAlreadyFalse;
        console.log(this.arlobotSvc.webModel[key]);
        if (this.arlobotSvc.webModel[key]) {
            send = valueIfAlreadyTrue;
        }
        this.arlobotSvc.emitValue(send);
    }

    public updaterosParametersKey(key, valueIfAlreadyTrue, valueIfAlreadyFalse):void {
        var send = valueIfAlreadyFalse;
        console.log(this.arlobotSvc.webModel['rosParameters'][key]);
        if (this.arlobotSvc.webModel['rosParameters'][key]) {
            send = valueIfAlreadyTrue;
        }
        this.arlobotSvc.emitValue(send);
    }

    public sendSpecificSignalToArlobot(signal):void {
        this.arlobotSvc.emitValue(signal);
    }

    public sendDataToArlobot(signal, data):void {
        this.arlobotSvc.sendData(signal, data);
    }

    public virtualJoystickFunction():void {
        var that = this;
        this.joystickOutput = 'Use finger or mouse to drive robot!';
        // var refreshInterval = 1 / 30 * 1000;
        const refreshInterval = 1 / 15 * 1000;
        const DECREASER = 100;
        var writeInputToScreen = function () {
            let deltaX = (Math.round((joystick.deltaX()/DECREASER)*100)/100);
            let deltaY = (Math.round((joystick.deltaY()/DECREASER)*100)/100);
            that.joystickOutput = ' dx:' + deltaX
                + ' dy:' + deltaY
                + (joystick.right() ? ' Right' : '')
                + (joystick.up() ? ' Forward' : '')
                + (joystick.left() ? ' Left' : '')
                + (joystick.down() ? ' Reverse' : '');
            if (that.rosSvc.connected) {
                that.rosSvc.sendTwistCommandToROS(-deltaY, -deltaX);
            }
        };
        var movementWatcher = function () {
            console.log('.');
            writeInputToScreen();
            movementTimeout = setTimeout(movementWatcher, refreshInterval);
        };
        var movementTimeout;
        console.log("touchscreen is", VirtualJoystick.touchScreenAvailable() ? "available" : "not available");

        var joystick = new VirtualJoystick({
            container: document.getElementById('virtual-joystick-container'),
            mouseSupport: true
        });
        joystick.addEventListener('touchStart', function () {
            // TODO: This doesn't work for mouse down, only touch.
            console.log('down')
            if (movementTimeout !== undefined) {
                clearTimeout(movementTimeout);
            }
            movementTimeout = setTimeout(movementWatcher, refreshInterval);
        });
        joystick.addEventListener('touchEnd', function () {
            console.log('up')
            if (movementTimeout !== undefined) {
                clearTimeout(movementTimeout);
            }
            // TODO: This doesn't reset to ZERO for some reason,
            // But it doesn't stop us from sending ZERO to the robot.
            writeInputToScreen();
            console.log(that.rosSvc);
            console.log(that.arlobotSvc);
            that.joystickOutput = 'Stopped';
            if (that.rosSvc.connected) {
                that.rosSvc.sendTwistCommandToROS(0.0, 0.0);
                // Send a few to make sure it stops ;)
                that.rosSvc.sendTwistCommandToROS(0.0, 0.0);
                that.rosSvc.sendTwistCommandToROS(0.0, 0.0);
            } else {
                console.log('Ros not running.');
            }
        });
        // setInterval(function () {
        //     var outputEl = document.getElementById('virtual-joystick-result');
        //     outputEl.innerHTML = '<b>Result:</b> '
        //         + ' dx:' + joystick.deltaX()
        //         + ' dy:' + joystick.deltaY()
        //         + (joystick.right() ? ' right' : '')
        //         + (joystick.up() ? ' up' : '')
        //         + (joystick.left() ? ' left' : '')
        //         + (joystick.down() ? ' down' : '')
        // }, 1 / 30 * 1000);
    }

    onInit() {
        console.log(this.arlobotSvc.webModel);
        this.arlobotSvc.getData();
    }
}
