#!/usr/bin/env node
'use strict';
const personalData = require('./personalData');
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
const WebSocketClient = require('websocket').client;
const pm2 = require('pm2');
const Arduino = require('./Arduino');
const arduino = new Arduino(true);

// TODO: Presently if the microphone you want to use is on a USB port that is controlled by a relay,
// TODO: It may be that the microphone is not available when MyCroft starts.
// TODO: At this point I start Mycroft outside of the robot, so really the fix for this is external to mycroft.

let connection; // Hold connection data.
const client = new WebSocketClient();

// Use this list to exclude message types from logging to the console.
// You can also turn off all logging if you aren't debugging.
// These types might be useful for other things too someday?
// TODO: Set up lights on robot to blink when the mouth or eyes of mycroft are supposed to move?
const ignoreMessageTypes = ['enclosure.mouth.viseme', 'enclosure.eyes.blink'];

const init = function () {

    client.on('connectFailed', (error) => {
        console.log('Mycroft Connect Error: ' + error.toString());
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
                let messageObject = JSON.parse(message.utf8Data);
                if (ignoreMessageTypes.indexOf(messageObject.type) === -1) {
                    // Use this console logging to debug and replicate Mycroft messages.
                    // Turn them off to reduce logging.
                    console.log(`MyCroft ${messageObject.type}: ${message.utf8Data}`);
                }
                if (messageObject.type === 'connected') { // recognizer_loop:wakeword
                    arduino.currentCommandArrayPoint = 0;
                    arduino.currentCommandArray = [
                        `${arduino.lightPattern.fillSolid},0,255,0,${arduino.pixel.LOOP_END}`
                    ];
                }
            }
        });
    });

    client.connect('ws://localhost:8181/core', '');

};

const injectText = function (text) {
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

        //{"type": "recognizer_loop:utterance", "data": {"lang": "en-us", "utterances": ["hi"]}, "context": null}
        connection.sendUTF(JSON.stringify({
            type: "recognizer_loop:utterance",
            data: {
                lang: "en-us",
                utterances: [text]
            },
            context: null
        }));
    } else {
        console.log('No MyCroft connection found.');
    }
};


const sayText = function (text) {
    if (connection && connection.connected) {
        // MyCroft Activity: '{"data": {"expect_response": false, "utterance": "this is some text to"}, "type": "speak", "context": null}'
        connection.sendUTF(JSON.stringify({
            data: {
                expect_response: false,
                utterance: text
            },
            type: "speak",
            context: null
        }));
    } else {
        console.log('No MyCroft connection found.');
    }
};


exports.init = init;
exports.injectText = injectText;
exports.sayText = sayText;
