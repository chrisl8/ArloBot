#!/usr/bin/env node
'use strict';
const personalData = require('./personalData');
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
const spawn = require('child_process').spawn;
const masterRelay = require('./MasterRelay');
const UsbRelay = require('./UsbRelayControl');
const usbRelay = new UsbRelay();
const WebSocketClient = require('websocket').client;
const pm2 = require('pm2');
const Arduino = require('./Arduino');
const arduino = new Arduino(true);

// TODO: Presently if the microphone you want to use is on a USB port that is controlled by a relay,
// TODO: It may be that the microphone is not available when MyCroft starts.
// TODO: At this point I start Mycroft outside of the robot, so really the fix for this is external to mycroft.

var connection; // Hold connection data.
var client = new WebSocketClient();

var init = function () {

    client.on('connectFailed', (error) => {
        console.log('Connect Error: ' + error.toString());
    });

    client.on('connect', (newConnection) => {
        connection = newConnection;
        console.log('WebSocket Client Connected');
        connection.on('error', function (error) {
            console.log("Connection Error: " + error.toString());
        });
        connection.on('close', function () {
            console.log('echo-protocol Connection Closed');
        });
        connection.on('message', function (message) {
            if (message.type === 'utf8') {
                console.log("MyCroft Activity: '" + message.utf8Data + "'");
                let messageObject = JSON.parse(message.utf8Data);
                console.log(messageObject.message_type);
                if (messageObject.message_type === 'connected') { // recognizer_loop:wakeword
                    arduino.currentCommandArrayPoint = 0;
                    arduino.currentCommandArray = [
                        `${arduino.lightPattern.fillSolid},0,255,0,${arduino.pixel.LOOP_END}`
                    ];
                }
            }
        });
    });

    client.connect('ws://localhost:8000/events/ws', '');

};

var injectText = function (text) {
    if (connection && connection.connected) {
        // If you just want to SEE what Mycroft puts on the bus, don't send anything,
        // Then you can just watch and use what it gives you as a template
        //
        // The message bus is documented here:
        // https://docs.mycroft.ai/overview/messagebus

        // To just speak any text use this:
        //connection.sendUTF(JSON.stringify({message_type: 'speak', context: null, metadata: {utterance: 'Hello world'}}));

        // To 'inject' something as if it was spoken to mycroft use this:
        // This is where text comes in after being deciphered by the STT service.
        connection.sendUTF(JSON.stringify({
            message_type: 'recognizer_loop:utterance',
            context: null,
            metadata: {
                utterances: [text]
            }
        }));
    } else {
        console.log('No MyCroft connection found.');
    }
};

exports.init = init;
exports.injectText = injectText;
