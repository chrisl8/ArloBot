const spawn = require('child_process').spawn;
const personalData = require('./personalData');
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
const robotModel = require('./robotModel');

/*
 To get the state of all relays:
 switch_relay_name.sh all state
 Use this to populate a list of relays in webModel,
 and also get their names from
 personalData.relays
 */

class UsbRelay {
  constructor() {
    this.dataHolder = '';
    robotModel.usbRelayControlBusy = false;
    this.script = `${__dirname}/../scripts/switch_relay_name.sh`;
    this.updateAllRelayState();
  }

  static findRelayName(relayNumber) {
    for (const key in personalData.relays) {
      if (personalData.relays.hasOwnProperty(key)) {
        if (personalData.relays[key] === relayNumber) {
          return key;
        }
      }
    }
    return 'empty';
  }

  updateAllRelayState() {
    if (
      !personalData.demoWebSite &&
      personalData.useUSBrelay &&
      !robotModel.usbRelayControlBusy
    ) {
      if (webModel.debugging && webModel.logOtherMessages) {
        console.log('Updating Relay Status');
      }
      robotModel.usbRelayControlBusy = true;
      const child = spawn(this.script, ['all', 'state']);
      child.stdout.on('data', (data) => {
        if (data !== '') {
          this.dataHolder += data;
        }
      });

      child.stderr.on('data', (data) => {
        console.log(`UsbRelay output stderr: ${data}`);
      });

      child.on('close', (code) => {
        if (code === null || code === 0) {
          const linesArray = this.dataHolder.split('\n');
          const relayStatusArray = [];
          for (let i = 0; i < linesArray.length; i++) {
            if (['ON', 'OFF'].indexOf(linesArray[i]) > -1) {
              relayStatusArray.push(linesArray[i]);
            }
          }
          // WARNING: Relays are 1 indexed, not 0!
          for (let i = 0; i < relayStatusArray.length; i++) {
            const relayNumber = i + 1;
            const relayState = relayStatusArray[i];
            const relayName = UsbRelay.findRelayName(relayNumber);
            webModelFunctions.publishRelayState(
              relayNumber,
              relayState,
              relayName,
            );
          }
          this.dataHolder = '';
        } else {
          console.log(`UsbRelay State collection failed with code: ${code}`);
          console.log(this.dataHolder);
        }
        robotModel.usbRelayControlBusy = false;
      });
    } else if (personalData.demoWebSite) {
      const demoRelayCount = 10;
      if (webModel.relays.length < demoRelayCount) {
        // Generate some "fake" relays for the web demo.
        for (let i = 0; i < demoRelayCount; i++) {
          // WARNING: Relays are 1 indexed, not 0!
          const relayNumber = i + 1;
          const relayName = UsbRelay.findRelayName(relayNumber);
          webModelFunctions.publishRelayState(relayNumber, 'OFF', relayName);
        }
      }
    }
  }

  toggle(relayNumber) {
    const relayObject = webModel.relays.find((x) => x.number === relayNumber);
    if (relayObject) {
      if (relayObject.relayOn) {
        this.switchRelay(relayNumber, 'off');
      } else {
        this.switchRelay(relayNumber, 'on');
      }
    }
  }

  switchRelay(relayNumber, onOrOff) {
    if (!personalData.demoWebSite) {
      const delay = 100; // milliseconds
      const timeout = 20; // loops
      let triedSoFar = 0;
      const checkBusyOrSwitch = () => {
        if (!robotModel.usbRelayControlBusy) {
          robotModel.usbRelayControlBusy = true;
          const state = onOrOff.toLowerCase();
          if (state !== 'on' && state !== 'off') {
            return;
          }
          const child = spawn(this.script, [relayNumber, state]);

          child.stderr.on('data', (data) => {
            console.log(`UsbRelay output stderr: ${data}`);
          });

          child.on('close', (code) => {
            if (code === null || code === 0) {
              webModelFunctions.scrollingStatusUpdate(
                `Relay ${relayNumber} ${state}`,
              );
            } else {
              webModelFunctions.scrollingStatusUpdate(
                `Relay ${relayNumber} FAILED code: ${code}`,
              );
              console.log(`UsbRelay Switch failed with code: ${code}`);
            }
            const relayName = UsbRelay.findRelayName(relayNumber);
            webModelFunctions.publishRelayState(
              relayNumber,
              onOrOff.toUpperCase(),
              relayName,
            );
            robotModel.usbRelayControlBusy = false;
          });
        } else {
          triedSoFar++;
          if (triedSoFar < timeout) {
            setTimeout(checkBusyOrSwitch, delay);
          }
        }
      };
      checkBusyOrSwitch();
    } else {
      const relayName = UsbRelay.findRelayName(relayNumber);
      let relayState = 'OFF';
      if (onOrOff.toLowerCase() === 'on') {
        relayState = 'ON';
      }
      webModelFunctions.publishRelayState(relayNumber, relayState, relayName);
    }
  }
}

module.exports = UsbRelay;

if (require.main === module) {
  const usbRelay = new UsbRelay();
  usbRelay.updateAllRelayState();
  setTimeout(() => {
    console.log(webModel.relays);
  }, 1000);
}
