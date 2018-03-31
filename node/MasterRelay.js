'use strict';
// Numato Lab - http://numato.com
// https://github.com/numato/samplecode/blob/master/RelayAndGPIOModules/USBRelayAndGPIOModules/node.js/usbrelay/UsbRelay.js
const process = require('process');
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
const UsbDevice = require('./UsbDevice.js');
const SerialPort = require("serialport");
const personalData = require('./personalData');
let working = false; // Prevent multiple instances from running at once in the same program

function getPortName() {
    /** @namespace personalData.masterPowerRelayStringLocation */
    /** @namespace personalData.masterPowerRelayUniqueString */
    const relayDevice = new UsbDevice(personalData.masterPowerRelayUniqueString, personalData.masterPowerRelayStringLocation);
    return relayDevice.findDeviceName();
}

function usbRelay(operation, runFromCommandLine) {
    const wrapUp = function (runFromCommandLine, error) {
        if (runFromCommandLine && error) {
            console.error('Failed to write to port: ' + error);
            process.exit(1);
        }
        working = false;
    };
    if (!personalData.demoWebSite && (operation !== 'read' || !working)) {
        working = true;
        getPortName()
            .then((port) => {
                if (webModel.debugging) {
                    console.log('Master Relay Port:', port);
                }
                if (webModel.debugging) {
                    console.log('Master Relay opeartion:', operation);
                }

                if (operation === 'toggle') {
                    if (!webModel.masterRelayOn) {
                        operation = 'on';
                    } else {
                        operation = 'off';
                    }
                }

                const portObj = new SerialPort(port, {
                    baudRate: 19200,
                    autoOpen: false
                });

                portObj.on('data', function (data) {
                        if (runFromCommandLine) {
                            // The "buffer" has to be converted into a string.
                            if (operation === 'read') {
                                let output = String(data).split('\n\r'); // Serial port uses \n\r for line breaks
                                console.log(output[1]);
                            } else {
                                console.log(String(data));
                            }
                        } else {
                            if (operation === 'read') {
                                let output = String(data).split('\n\r'); // Serial port uses \n\r for line breaks
                                let result = false;
                                if (output[1] === 'on') {
                                    result = true;
                                }
                                webModelFunctions.update('masterRelayOn', result);
                            } else {
                                usbRelay('read');
                            }
                        }
                        portObj.close();
                        wrapUp();
                    }
                );

                portObj.open(error => {
                    if (error) {
                        wrapUp(runFromCommandLine, error);
                    } else {
                        if (operation === 'read') {
                            portObj.write("relay read 0\r", function (err) { // Argument Options: err, result
                                if (err) {
                                    wrapUp(runFromCommandLine, err);
                                }
                            });
                        } else {
                            webModelFunctions.scrollingStatusUpdate(`Switching Master Relay ${operation}.`);
                            portObj.write("relay " + operation + " 0\r", function (err) { // Argument Options: err, result
                                if (err) {
                                    wrapUp(runFromCommandLine, err);
                                }
                            });
                        }
                    }
                });
            })
            .catch(error => {
                wrapUp(runFromCommandLine, error);
            });
    } else if (personalData.demoWebSite) {
        if (operation === 'toggle') {
            if (!webModel.masterRelayOn) {
                webModelFunctions.update('masterRelayOn', true);
            } else {
                webModelFunctions.update('masterRelayOn', false);
            }
        }
    }
}
module.exports = usbRelay;

if (require.main === module) {
    // Run the function if this is called directly instead of required as a module.
    if (process.argv.length < 3) {
        console.log("You must provide an operation. One of: on, off or read, for example:");
        console.log("node MasterRelay.js off");
        process.exit();
    }
    const operation = process.argv[2];
    usbRelay(operation, true);
}
