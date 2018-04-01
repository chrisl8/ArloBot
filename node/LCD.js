const process = require('process');
const UsbDevice = require('./UsbDevice.js');
const SerialPort = require('serialport');
const personalData = require('./personalData');

let working = false; // Prevent multiple instances from running at once in the same program

function getPortName() {
  /** @namespace personalData.masterPowerRelayStringLocation */
  /** @namespace personalData.masterPowerRelayUniqueString */
  const relayDevice = new UsbDevice(
    personalData.LCDString,
    personalData.LCDLocation,
  );
  return relayDevice.findDeviceName();
}

// https://learn.adafruit.com/usb-plus-serial-backpack/command-reference
const commandList = {
  displayOn: 0x42,
  displayOff: 0x46,
  clear: 0x58,
  autoscrollOn: 0x51,
  autoscrollOff: 0x52,
  cursorHome: 0x48,
  cursorBack: 0x4c,
  cursorForward: 0x4d,
  underlineCursorOn: 0x4a,
  underlineCursorOff: 0x4b,
  blockCursorOn: 0x53,
  blockCursorOff: 0x54,
};

/*
 Moving and changing the cursor:
 Set cursor position - 0xFE 0x47 - set the position of text entry cursor. Column and row numbering starts with 1 so the first position in the very top left is (1, 1)
 */

function LCD({ operation, input, runFromCommandLine, row, red, green, blue }) {
  const wrapUp = ({ error, portObj }) => {
    if (portObj) {
      portObj.close();
    }
    if (runFromCommandLine && error) {
      console.error(`Failed to write to port: ${error}`);
      process.exit(1);
    }
    working = false;
  };
  if (!working && (runFromCommandLine || personalData.useLCD)) {
    working = true;
    getPortName()
      .then((port) => {
        const portObj = new SerialPort(port, {
          baudRate: 19200,
          autoOpen: false,
        });

        portObj.open((error) => {
          if (error) {
            wrapUp({ runFromCommandLine, error });
          } else if (commandList.hasOwnProperty(operation)) {
            portObj.write(
              Buffer.from([0xfe, commandList[operation]]),
              (err) => {
                // Argument Options: err, result
                if (err) {
                  wrapUp({
                    runFromCommandLine,
                    error: err,
                    portObj,
                  });
                } else {
                  wrapUp({
                    runFromCommandLine,
                    portObj,
                  });
                }
              },
            );
          } else {
            switch (operation) {
              case 'text': {
                let output = input;
                // Make input match a full line length to avoid leaving garbage behind.
                while (output.length < 15) {
                  // eslint-disable-next-line no-param-reassign
                  output = `${output} `;
                }

                const outputArray = [0xfe, 0x47, 1];
                if (row === 'bottom') {
                  outputArray.push(2);
                } else {
                  outputArray.push(1);
                }

                portObj.write(Buffer.from(outputArray), (err) => {
                  // Argument Options: err, result
                  if (err) {
                    wrapUp({
                      runFromCommandLine,
                      error: err,
                      portObj,
                    });
                  } else {
                    portObj.drain(() => {
                      portObj.write(output.slice(0, 16), (e) => {
                        // Argument Options: err, result
                        if (e) {
                          wrapUp({
                            runFromCommandLine,
                            error: e,
                            portObj,
                          });
                        } else {
                          wrapUp({
                            runFromCommandLine,
                            portObj,
                          });
                        }
                      });
                    });
                  }
                });
                break;
              }
              case 'hex':
                portObj.write(Buffer.from([0xfe, input]), (err) => {
                  // Argument Options: err, result
                  if (err) {
                    wrapUp({
                      runFromCommandLine,
                      error: err,
                      portObj,
                    });
                  } else {
                    wrapUp({
                      runFromCommandLine,
                      portObj,
                    });
                  }
                });
                break;
              case 'color':
                portObj.write(
                  Buffer.from([0xfe, 0xd0, red, green, blue]),
                  (err) => {
                    // Argument Options: err, result
                    if (err) {
                      wrapUp({
                        runFromCommandLine,
                        error: err,
                        portObj,
                      });
                    } else {
                      wrapUp({
                        runFromCommandLine,
                        portObj,
                      });
                    }
                  },
                );
                break;
              case 'brightness':
                portObj.write(Buffer.from([0xfe, 0x99, input]), (err) => {
                  // Argument Options: err, result
                  if (err) {
                    wrapUp({
                      runFromCommandLine,
                      error: err,
                      portObj,
                    });
                  } else {
                    wrapUp({
                      runFromCommandLine,
                      portObj,
                    });
                  }
                });
                break;
              case 'contrast':
                portObj.write(Buffer.from([0xfe, 0x50, input]), (err) => {
                  // Argument Options: err, result
                  if (err) {
                    wrapUp({
                      runFromCommandLine,
                      error: err,
                      portObj,
                    });
                  } else {
                    wrapUp({
                      runFromCommandLine,
                      portObj,
                    });
                  }
                });
                break;
              default:
                wrapUp({
                  runFromCommandLine,
                  error: 'Unknown command.',
                });
            }
          }
        });
      })
      .catch((error) => {
        wrapUp({ runFromCommandLine, error });
      });
  }
}

module.exports = LCD;

if (require.main === module) {
  // Run the function if this is called directly instead of required as a module.
  if (process.argv.length < 3) {
    console.log(
      'You must provide command line parameters and operations. Here are examples:',
    );
    console.log('Display text on the screen:');
    console.log("node LCD.js text 'Test'");
    console.log('Display text on the screen and select a row:');
    console.log("node LCD.js text 'top' top");
    console.log("node LCD.js text 'bottom' bottom");
    console.log('Send a specific hex code command:');
    console.log('node LCD.js hex 0x58');
    console.log(
      'Change the background light color. <red> <green> <blue> are numbers from 0 to 255:',
    );
    console.log('node LCD.js color <red> <green> <blue>');
    console.log('Along with some specific commands:');
    console.log('node LCD.js displayOn');
    console.log('node LCD.js displayOff');
    console.log('node LCD.js clear');
    console.log('node LCD.js brightness <number 0 to 255>');
    console.log('node LCD.js contrast <number 0 to 255>');
    process.exit();
  }
  const operation = process.argv[2];
  const input = process.argv[3];
  let row;
  if (operation === 'text') {
    row = process.argv[4];
  }
  let red;
  let green;
  let blue;
  if (operation === 'color') {
    red = process.argv[3];
    green = process.argv[4];
    blue = process.argv[5];
  }
  LCD({
    operation,
    input,
    runFromCommandLine: true,
    row,
    red,
    green,
    blue,
  });
}
