/* eslint-disable no-param-reassign */
// Numato Lab - http://numato.com
// https://github.com/numato/samplecode/blob/master/RelayAndGPIOModules/USBRelayAndGPIOModules/node.js/usbrelay/UsbRelay.js
const SerialPort = require('serialport');
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
const UsbDevice = require('./UsbDevice.js');
const personalData = require('./personalData');

let working = false; // Prevent multiple instances from running at once in the same program

function getPortName() {
  /** @namespace personalData.masterPowerRelayStringLocation */
  /** @namespace personalData.masterPowerRelayUniqueString */
  const relayDevice = new UsbDevice(
    personalData.masterPowerRelayUniqueString,
    personalData.masterPowerRelayStringLocation,
  );
  return relayDevice.findDeviceName();
}

function usbRelay(operation, runFromCommandLine) {
  const wrapUp = (wrapUpRunFromCommandLine, error) => {
    if (wrapUpRunFromCommandLine && error) {
      console.error(`Failed to write to port: ${error}`);
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
          console.log('Master Relay operation:', operation);
        }

        if (operation === 'toggle') {
          if (!webModel.masterRelayOn) {
            // noinspection AssignmentToFunctionParameterJS
            operation = 'on';
          } else {
            // noinspection AssignmentToFunctionParameterJS
            operation = 'off';
          }
        }

        const portObj = new SerialPort(port, {
          baudRate: 19200,
          autoOpen: false,
        });

        portObj.on('data', (data) => {
          if (runFromCommandLine) {
            // The "buffer" has to be converted into a string.
            if (operation === 'read') {
              const output = String(data).split('\n\r'); // Serial port uses \n\r for line breaks
              console.log(output[1]);
            } else {
              console.log(String(data));
            }
          } else if (operation === 'read') {
            const output = String(data).split('\n\r'); // Serial port uses \n\r for line breaks
            let result = false;
            if (output[1] === 'on') {
              result = true;
            }
            if (webModel.masterRelayOn !== result) {
              // Helps us see what is going on,
              // and resets the idle timer if somebody flips the relay on/off
              // via an external script.
              webModelFunctions.scrollingStatusUpdate(
                `Master Relay ${output[1]}`,
              );
            }
            webModelFunctions.update('masterRelayOn', result);
          } else {
            usbRelay('read');
          }
          portObj.close();
          wrapUp();
        });

        portObj.open((error) => {
          if (error) {
            wrapUp(runFromCommandLine, error);
          } else if (operation === 'read') {
            portObj.write('relay read 0\r', (err) => {
              // Argument Options: err, result
              if (err) {
                wrapUp(runFromCommandLine, err);
              }
            });
          } else {
            webModelFunctions.scrollingStatusUpdate(
              `Switching Master Relay ${operation}.`,
            );
            portObj.write(`relay ${operation} 0\r`, (err) => {
              // Argument Options: err, result
              if (err) {
                wrapUp(runFromCommandLine, err);
              }
            });
          }
        });
      })
      .catch((error) => {
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
    console.log(
      'You must provide an operation. One of: on, off or read, for example:',
    );
    console.log('node MasterRelay.js off');
    process.exit();
  }
  const operation = process.argv[2];
  usbRelay(operation, true);
}
