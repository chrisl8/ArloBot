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
        // const relayDevice = new UsbDevice(personalData.masterPowerRelayUniqueString, personalData.masterPowerRelayStringLocation);
    const relayDevice = new UsbDevice("FT230X_Basic_UART", "ID_MODEL");
    return relayDevice.findDeviceName();
}

function ScanseSweep(operation, runFromCommandLine) {
    const wrapUp = function (runFromCommandLine, error) {
        if (runFromCommandLine && error) {
            console.error('Failed to write to port: ' + error);
            process.exit(1);
        }
        working = false;
    };
    if (!personalData.demoWebSite && !working) {
        working = true;
        getPortName()
            .then((port) => {

                const portObj = new SerialPort(port, {
                    baudRate: 115200,
                    autoOpen: false
                });

                portObj.on('data', function (data) {
                        if (runFromCommandLine) {
                            // The "buffer" has to be converted into a string.
                            console.log(String(data));
                        }
                        portObj.close();
                        wrapUp();
                    }
                );

                portObj.open(error => {
                    if (error) {
                        wrapUp(runFromCommandLine, error);
                    } else {
                        if (operation === 'stop') {
                            console.log('Stoping Scanse Sweep.');
                            webModelFunctions.scrollingStatusUpdate('Stoping Scanse Sweep.');
                            portObj.write("MS00\r", function (err) { // Argument Options: err, result
                                if (err) {
                                    wrapUp(runFromCommandLine, err);
                                }
                            });
                        } else {
                            webModelFunctions.scrollingStatusUpdate(`Sending command ${operation} to Scanse Sweep.`);
                            portObj.write(`${operation}\r`, function (err) { // Argument Options: err, result
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
    }
}

module.exports = ScanseSweep;

if (require.main === module) {
    // Run the function if this is called directly instead of required as a module.
    if (process.argv.length < 3) {
        console.log("You must provide an operation. One of: stop or a Scanse Sweep command:");
        console.log("node ScanseSweepControl.js stop");
        console.log("node ScanseSweepControl.js MS05");
        process.exit();
    }
    const operation = process.argv[2];
    ScanseSweep(operation, true);
}
