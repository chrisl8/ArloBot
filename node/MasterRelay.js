/* eslint-disable no-param-reassign */
// Numato Lab - http://numato.com
// https://github.com/numato/samplecode/blob/master/RelayAndGPIOModules/USBRelayAndGPIOModules/node.js/usbrelay/UsbRelay.js
const SerialPort = require('serialport');
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
const UsbDevice = require('./UsbDevice.js');
const personalData = require('./personalData');
const robotModel = require('./robotModel');
const handleSemaphoreFiles = require('./handleSemaphoreFiles');

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
    robotModel.masterRelayBusy = false;
    if (operation !== 'read') {
      if (runFromCommandLine) {
        // If this is run from the command line, the Robot server can't see the result,
        // so use the semaphore file to tell it to update.
        handleSemaphoreFiles.setSemaphoreFiles('checkMasterRelayFile');
      } else {
        // Force a read to ensure the change worked,
        // and update the webModel with the new state.
        usbRelay('read');
      }
    }
  };
  if (
    !personalData.demoWebSite &&
    !robotModel.usbRelayControlBusy &&
    !robotModel.masterRelayBusy
  ) {
    robotModel.masterRelayBusy = true;
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
