// Numato Lab - http://numato.com
// https://github.com/numato/samplecode/blob/master/RelayAndGPIOModules/USBRelayAndGPIOModules/node.js/usbrelay/UsbRelay.js
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
const UsbDevice = require('./UsbDevice.js');
const SerialPort = require("serialport").SerialPort;
var personalData = require('./personalData');
var working = false; // Prevent multiple instances from running at once in the same program

var getPortName = function () {
    var relayDevice = new UsbDevice(personalData.masterPowerRelayUniqueString, personalData.masterPowerRelayStringLocation);
    return relayDevice.findDeviceName();
};

var usbRelay = function (operation, runFromCommandLine) {
    if (operation !== 'read' || !working) {
        working = true;
        getPortName()
            .then((port) => {

                var wrapUp = function (runFromCommandLine, error) {
                    if (runFromCommandLine && error) {
                        console.error('Failed to write to port: ' + err);
                        process.exit(1);
                    }
                    working = false;
                };

                if (operation === 'toggle') {
                    if (!webModel.masterRelayOn) {
                        operation = 'on';
                    } else {
                        operation = 'off';
                    }
                }

                var portObj = new SerialPort(port, {
                    baudrate: 19200
                }, false);

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
                            }
                        }
                        portObj.close();
                        wrapUp();
                    }
                );

                portObj.open(function (error) {
                    if (error) {
                        console.log('Master Relay Error: ' + error);
                    } else {
                        if (operation === 'read') {
                            portObj.write("relay read 0\r", function (err, result) {
                                if (err) {
                                    wrapUp(runFromCommandLine, err);
                                }
                            });
                        } else {
                            webModelFunctions.scrollingStatusUpdate(`Switching Master Relay ${operation}.`)
                            portObj.write("relay " + operation + " 0\r", function (err, results) {
                                if (err) {
                                    wrapUp(runFromCommandLine, err);
                                }
                            });
                        }
                    }
                });
            });
    }
};
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
