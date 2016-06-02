var personalData = require('./personalData');
var webModel = require('./webModel');
var webModelFunctions = require('./webModelFunctions');
var robotModel = require('./robotModel');
const spawn = require('child_process').spawn;
var ipAddress = require('./ipAddress');

/*
 To get the state of all relays:
 switch_relay_name.sh all state
 Use this to populate a list of relays in webModel,
 and also get their names from
 personalData.relays
 */

module.exports = class UsbRelay {
    constructor() {
        this.dataHolder = '';
        this.busy = false;
        this.script = __dirname + '/../scripts/switch_relay_name.sh';
    }

    findRelayName(relayNumber) {
        for (let key in personalData.relays) {
            if (personalData.relays.hasOwnProperty(key)) {
                if (personalData.relays[key] === relayNumber) {
                    return key;
                }
            }
        }
        return 'empty';
    }

    updateAllRelayState() {
        if (!this.busy) {
            this.busy = true;
            const process = spawn(this.script, ['all', 'state']);
            process.stdout.on('data', (data) => {
                if (data != '') {
                    this.dataHolder += data;
                }
            });

            process.stderr.on('data', (data) => {
                console.log(`UsbRelay output stderr: ${data}`);
            });

            process.on('close', (code) => {
                if (code === null || code === 0) {
                    let linesArray = this.dataHolder.split('\n');
                    let relayStatusArray = [];
                    for (let i = 0; i < linesArray.length; i++) {
                        if (['ON', 'OFF'].indexOf(linesArray[i]) > -1) {
                            relayStatusArray.push(linesArray[i])
                        }
                    }
                    // WARNING: Relays are 1 indexed, not 0!
                    for (let i = 0; i < relayStatusArray.length; i++) {
                        let relayNumber = i + 1;
                        let relayState = relayStatusArray[i];
                        let relayName = this.findRelayName(relayNumber);
                        webModelFunctions.publishRelayState(relayNumber, relayState, relayName);
                    }
                    this.dataHolder = '';
                } else {
                    console.log(`UsbRelay State collection failed with code: ${code}`);
                }
                this.busy = false;
            });
        }
    }

    toggle(relayNumber) {
        const relayObject = webModel.relays.find(x=> x.number === relayNumber);
        if (relayObject) {
            if (relayObject.relayOn) {
                this.switch(relayNumber, 'off');
            } else {
                this.switch(relayNumber, 'on');
            }
        }
    }

    switch(relayNumber, onOrOff) {
        const state = onOrOff.toLowerCase();
        if (state !== 'on' && state !== 'off') {
            return;
        }
        const process = spawn(this.script, [relayNumber, state]);

        process.stderr.on('data', (data) => {
            console.log(`UsbRelay output stderr: ${data}`);
        });

        process.on('close', (code) => {
            if (code === null || code === 0) {
                webModelFunctions.scrollingStatusUpdate(`Relay ${relayNumber} ${state}`);
            } else {
                console.log(`UsbRelay State collection failed with code: ${code}`);
            }
        });
    }

};
