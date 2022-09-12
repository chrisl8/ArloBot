/*
 Replace the array this.currentCommandArray with an array of commands for the arduino neopixel controller,
 and this will loop over them continuously.
 Set this.currentCommandArrayPoint to 0 to put it back at command 0.
 */

const { SerialPort } = require('serialport');
const webModel = require('./webModel');
const robotModel = require('./robotModel');
const webModelFunctions = require('./webModelFunctions');
const UsbDevice = require('./UsbDevice');
const howManySecondsSince = require('./howManySecondsSince');
const masterRelay = require('./MasterRelay');
const personalData = require('./personalData');
const wait = require('./wait');

// Adjustable constants for the board:
const baudrate = 115200;
const retrySendDelay = 15;
const postSendDelay = 250;

class Arduino {
  /** @namespace personalData.relays.has_arduino */

  /** @namespace personalData.arduinoUniqueString */
  /** @namespace personalData.arduinoStringLocation */
  constructor(runFromCommandLine) {
    this.arduinoBusyTimeoutSeconds = 60;
    this.runFromCommandLine = runFromCommandLine;

    // Commands programmed into the arduino
    this.lightPattern = {
      colorWipe: 0,
      rainbow: 1,
      rainbowCycle: 2,
      theaterChase: 3,
      theaterChaseRainbow: 4,
      alternate: 5,
      fillSolid: 9,
    };

    // Named pixels
    this.pixel = {
      LOOP_START: 33, // First LED on the top of the ring.
      LOOP_END: 119, // My last pixel has no green :(
    };

    this.ledTestCommandArray = [
      // Clear the loop with a black wipe
      `${this.lightPattern.colorWipe},0,0,0,${this.pixel.LOOP_START},${this.pixel.LOOP_END},25`,
      // Start wipe in the middle of the loop
      `${this.lightPattern.colorWipe},255,255,255,100,${this.pixel.LOOP_END},25`,
      `${this.lightPattern.colorWipe},255,255,255,${this.pixel.LOOP_START},100,25`,
      // Wipe and clear small section in reverse order
      `${this.lightPattern.colorWipe},255,0,0,90,105,20`,
      `${this.lightPattern.colorWipe},0,0,0,105,90,20`,
      `${this.lightPattern.colorWipe},255,0,0,105,90,20`,
      `${this.lightPattern.colorWipe},0,0,0,90,105,20`,
      // Wipe and clear small section in same order
      `${this.lightPattern.colorWipe},255,0,0,90,105,20`,
      `${this.lightPattern.colorWipe},0,0,0,90,105,20`,
      `${this.lightPattern.colorWipe},255,0,0,105,90,20`,
      `${this.lightPattern.colorWipe},0,0,0,105,90,20`,
      // Theater Chase in a small section
      `${this.lightPattern.theaterChase},255,0,0,50,90,106,30`,
      // Clear loop with a black wipe
      `${this.lightPattern.colorWipe},0,0,0,${this.pixel.LOOP_START},${this.pixel.LOOP_END},25`,
      // Make some big color loops with escalating speed
      `${this.lightPattern.colorWipe},255,0,0,${this.pixel.LOOP_START},${this.pixel.LOOP_END},25`,
      `${this.lightPattern.colorWipe},0,255,0,${this.pixel.LOOP_START},${this.pixel.LOOP_END},15`,
      `${this.lightPattern.colorWipe},0,0,255,${this.pixel.LOOP_START},${this.pixel.LOOP_END},10`,
      `${this.lightPattern.rainbow},${this.pixel.LOOP_START},${this.pixel.LOOP_END},1`,
      `${this.lightPattern.rainbowCycle},${this.pixel.LOOP_START},${this.pixel.LOOP_END},1`,
      // Escalate the chase
      // NOTE: Use consecutive chases to evaluate update speed,
      // Is it taking too long for a new command to get to the board,
      // causing gaps in the animation, or is it fluid?
      `${this.lightPattern.theaterChase},170,170,170,50,${this.pixel.LOOP_START},${this.pixel.LOOP_END},5`,
      `${this.lightPattern.theaterChase},170,170,170,50,${this.pixel.LOOP_START},${this.pixel.LOOP_END},5`,
      `${this.lightPattern.theaterChase},170,170,170,50,${this.pixel.LOOP_START},${this.pixel.LOOP_END},5`,
      `${this.lightPattern.theaterChase},170,170,170,40,${this.pixel.LOOP_START},${this.pixel.LOOP_END},20`,
      `${this.lightPattern.theaterChase},170,170,170,30,${this.pixel.LOOP_START},${this.pixel.LOOP_END},30`,
      `${this.lightPattern.theaterChase},170,170,170,20,${this.pixel.LOOP_START},${this.pixel.LOOP_END},30`,
      // Reverse Chase
      `${this.lightPattern.theaterChase},170,170,170,15,${this.pixel.LOOP_END},${this.pixel.LOOP_START},40`,
      // Super fast!
      `${this.lightPattern.theaterChase},170,170,170,10,${this.pixel.LOOP_START},${this.pixel.LOOP_END},50`,
      `${this.lightPattern.theaterChase},170,170,170,5,${this.pixel.LOOP_START},${this.pixel.LOOP_END},55`,
      `${this.lightPattern.theaterChaseRainbow},50,${this.pixel.LOOP_START},${this.pixel.LOOP_END},256`,
    ];
    this.currentCommandArray = [];
  }

  static getPortName() {
    /** @namespace personalData.masterPowerRelayStringLocation */
    /** @namespace personalData.masterPowerRelayUniqueString */
    const arduinoDevice = new UsbDevice(
      personalData.arduinoUniqueString,
      personalData.arduinoStringLocation,
    );
    return arduinoDevice.findDeviceName();
  }

  turnOnRelays() {
    return new Promise((resolve, reject) => {
      let delay = 0;
      if (personalData.useMasterPowerRelay && !webModel.masterRelayOn) {
        masterRelay('on');
        delay = 3000;
      }
      setTimeout(() => {
        let delayTwo = 0;
        if (
          personalData.relays.has_arduino &&
          webModel.relays.find((x) => x.name === 'arduino') &&
          !webModel.relays.find((x) => x.name === 'arduino').relayOn
        ) {
          robotModel.usbRelay.switchRelay(
            webModel.relays.find((x) => x.name === 'arduino').number,
            'on',
          );
          delayTwo = 2000;
        }
        if (
          personalData.relays.has_fiveVolt &&
          webModel.relays.find((x) => x.name === 'fiveVolt') &&
          !webModel.relays.find((x) => x.name === 'fiveVolt').relayOn
        ) {
          robotModel.usbRelay.switchRelay(
            webModel.relays.find((x) => x.name === 'fiveVolt').number,
            'on',
          );
          delayTwo = 4000;
        }
        setTimeout(() => {
          // TODO: This assumes both relays exist.
          // TODO: Rely on results to continue instead of delay and fail.
          let relayOn = true;

          if (
            personalData.relays.has_arduino &&
            webModel.relays.find((x) => x.name === 'arduino') &&
            !webModel.relays.find((x) => x.name === 'arduino').relayOn
          ) {
            relayOn = false;
          }
          if (
            personalData.relays.has_fiveVolt &&
            webModel.relays.find((x) => x.name === 'fiveVolt') &&
            !webModel.relays.find((x) => x.name === 'fiveVolt').relayOn
          ) {
            relayOn = false;
          }

          if (relayOn) {
            resolve();
          } else {
            reject(new Error('Relays did not turn on.'));
          }
        }, delayTwo);
      }, delay);
    });
  }

  sendCommandToArduino() {
    if (this.repeatTimeout) {
      clearTimeout(this.repeatTimeout);
    }
    if (
      !this.arduinoReady ||
      (this.arduinoBusy &&
        howManySecondsSince(this.arduinoBusyStartTime) <
          this.arduinoBusyTimeoutSeconds)
    ) {
      this.repeatTimeout = setTimeout(() => {
        this.sendCommandToArduino();
      }, retrySendDelay);
    } else {
      this.arduinoBusy = false;
      this.arduinoBusyStartTime = new Date();
      if (
        this.runFromCommandLine ||
        (webModel.debugging && webModel.logOtherMessages)
      ) {
        console.log(
          `${this.currentCommandArray[this.currentCommandArrayPoint]}->`,
        );
      }
      this.portObj.write(
        `${this.currentCommandArray[this.currentCommandArrayPoint]}\n`,
        (err) => {
          // Argument Options: err, result
          if (err) {
            console.log(`Write ERROR: ${err}`);
          }
          this.lastSentData = `${
            this.currentCommandArray[this.currentCommandArrayPoint]
          }`;
          if (
            this.currentCommandArrayPoint <
            this.currentCommandArray.length - 1
          ) {
            this.currentCommandArrayPoint++;
          } else {
            this.currentCommandArrayPoint = 0;
          }
          this.repeatTimeout = setTimeout(() => {
            this.sendCommandToArduino();
          }, postSendDelay);
        },
      );
    }
  }

  init() {
    // See Camera.js for example
    let inputStream = '';
    const inputArray = [];
    this.currentCommandArray = this.ledTestCommandArray;
    if (!this.programIsBusy) {
      webModelFunctions.update('neoPixelsOn', true);
      this.programIsBusy = true;
      this.turnOnRelays()
        .then(() => Arduino.getPortName())
        .then((port) => {
          this.currentCommandArrayPoint = 0;

          // var wrapUp = (runFromCommandLine, error) => {
          //     if (runFromCommandLine && error) {
          //         console.error('Failed to write to port: ' + error);
          //         process.exit(1);
          //     }
          //     this.programIsBusy = false;
          // };

          this.portObj = new SerialPort({
            path: port,
            baudRate: baudrate,
            autoOpen: false,
          });

          this.portObj.on('data', (data) => {
            // The "buffer" has to be converted into a string.
            // console.log(String(data));
            inputStream += String(data);
            // Serial Port uses \r\n to separate lines of input.
            if (inputStream.indexOf('\r\n') > -1) {
              inputArray.push(inputStream.split('\r\n'));
              inputStream = '';
              const output = inputArray.shift();
              if (
                this.runFromCommandLine ||
                (webModel.debugging && webModel.logOtherMessages)
              ) {
                console.log(`<-${output[0]}`);
              }
              if (output[0] === 'BUSY') {
                this.arduinoBusy = true;
                this.arduinoBusyStartTime = new Date();
              }
              if (output[0] === 'READY') {
                if (this.repeatTimeout) {
                  clearTimeout(this.repeatTimeout);
                }
                this.repeatTimeout = setTimeout(() => {
                  this.sendCommandToArduino();
                }, postSendDelay);
                this.arduinoBusy = false;
                this.arduinoReady = true;
              }
              if (output[0] === 'DONE') {
                if (this.repeatTimeout) {
                  clearTimeout(this.repeatTimeout);
                }
                this.repeatTimeout = setTimeout(() => {
                  this.sendCommandToArduino();
                }, retrySendDelay);
                this.arduinoBusy = false;
              }
            }
          });

          this.portObj.on('close', () => {
            this.pause();
            this.portObj = null;
            this.programIsBusy = false;
            webModelFunctions.update('neoPixelsOn', false);
          });

          this.portObj.open((error) => {
            if (error) {
              console.log(`Arduino Connection Error: ${error}`);
            } else {
              this.repeatTimeout = setTimeout(() => {
                this.sendCommandToArduino();
              }, retrySendDelay);
            }
          });
        })
        .catch((e) => {
          console.log(`Arduino Error: ${e.message}`);
          webModelFunctions.update('neoPixelsOn', false);
          this.programIsBusy = false;
        });
    }
  }

  pause() {
    if (this.repeatTimeout) {
      clearTimeout(this.repeatTimeout);
    }
  }

  waitUntilNotBusy(callback) {
    setTimeout(() => {
      if (!this.arduinoBusy) {
        callback();
      } else {
        setTimeout(() => {
          this.waitUntilNotBusy(callback);
        }, 1000);
      }
    }, 1000);
  }

  async lightsOut() {
    this.currentCommandArrayPoint = 0;
    this.currentCommandArray = [
      `${this.lightPattern.fillSolid},0,0,0,${this.pixel.LOOP_END}`,
    ];
    // Wait for clear command to actually get sent before shutting down
    // with maximum wait timer
    let waitCountdown = 30;
    while (
      this.lastSentData !==
        `${this.lightPattern.fillSolid},0,0,0,${this.pixel.LOOP_END}` &&
      waitCountdown > 0
    ) {
      if (
        this.runFromCommandLine ||
        (webModel.debugging && webModel.logOtherMessages)
      ) {
        console.log('Waiting for Arduino to turn lights off:');
        console.log(this.lastSentData);
        console.log(
          `${this.lightPattern.fillSolid},0,0,0,${this.pixel.LOOP_END}`,
        );
      }
      // eslint-disable-next-line no-await-in-loop
      await wait(1);
      waitCountdown--;
    }
    await wait(1); // On last wait to ensure the commands had time to get through.
    if (this.portObj) {
      this.portObj.close();
    }
    this.programIsBusy = false;
    webModelFunctions.update('neoPixelsOn', false);
  }
}

module.exports = Arduino;
if (require.main === module) {
  // Run the function if this is called directly instead of required as a module.
  const arduino = new Arduino(true);
  arduino.init();
  // Test sending custom pattern lists:
  // setTimeout(()=> {
  //     arduino.currentCommandArray = [
  //         `${arduino.lightPattern.colorWipe},255,0,0,${arduino.pixel.LOOP_START},${arduino.pixel.LOOP_END},10`,
  //         `${arduino.lightPattern.colorWipe},0,255,0,${arduino.pixel.LOOP_END},${arduino.pixel.LOOP_START},10`,
  //         `${arduino.lightPattern.colorWipe},0,0,255,${arduino.pixel.LOOP_START},${arduino.pixel.LOOP_END},10`
  //     ];
  //     arduino.currentCommandArrayPoint = 0;
  //     console.log('Now it should be different.');
  //     setTimeout(()=>{
  //         arduino.currentCommandArray = arduino.ledTestCommandArray;
  //         arduino.currentCommandArrayPoint = 0;
  //         console.log('Now back to the test pattern');
  //     }, 30000);
  // }, 20000);

  // Test pausing:
  // setTimeout(()=> {
  //     arduino.pause();
  //     console.log('Lights should cease shortly.');
  // }, 30000);

  // Test shutting off lights:
  // setTimeout(()=> {
  //    console.log('Lights should go out and stay out shortly.');
  //    arduino.lightsOut();
  // }, 30000);
}
