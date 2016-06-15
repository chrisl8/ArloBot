const personalData = require('./personalData');
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
const robotModel = require('./robotModel');
const fs = require('fs');
const exec = require('child_process').exec;
const say = require('say');
const push = require('pushover-notifications');

module.exports = function(sound) {

    // This will not make any noise if the file
    // ~/.arlobot/status/bequiet
    // exists
    var beQuietFile = process.env.HOME + '/.arlobot/status/bequiet';
    fs.open(beQuietFile, 'r', function(err) {
        if (err) {
            console.log(sound);

            // Set volume at max
            var setVolumeCommand = '/usr/bin/amixer set Master ' + personalData.speechVolumeLevelDefault + '% on';
            exec(setVolumeCommand);

            // Turn on external speaker if off and plugged in
            //   At least this will help keep it charged. ;)
            // if (personalData.use_external_speaker && personalData.useMasterPowerRelay && personalData.relays.has_fiveVolt && webModel.pluggedIn && webModel.relays.find(x=> x.name === 'fiveVolt') && !webModel.relays.find(x=> x.name === 'fiveVolt')['relayOn'] && !robotModel.master.isAsleep) {
            //     if (!webModel.masterRelayOn) {
            //         const masterRelay = require('./MasterRelay');
            //         masterRelay('on');
            //     }
            //     const UsbRelay = require('./UsbRelayControl');
            //     var usbRelay = new UsbRelay();
            //     if (webModel.relays.find(x=> x.name === 'fiveVolt') && !webModel.relays.find(x=> x.name === 'fiveVolt')['relayOn']) {
            //         usbRelay.switch(webModel.relays.find(x=> x.name === 'fiveVolt')['number'],'on');
            //     }
            // }

            // This script can accept text to speak,
            // or .wav files to play.
            // We rely stricly on the extension to
            // decide what it is!
            var possibleExtension = sound.slice(-4).toLowerCase();
            if (possibleExtension === '.wav') {
                exec('/usr/bin/aplay -q ' + sound);
            } else {
                if (personalData.speechProgram === 'nodeSay') {
                    // https://github.com/marak/say.js/
                    // no callback, fire and forget
                    say.speak(sound);
                } else if (personalData.speechProgram) {
                    var speechCommand = personalData.speechProgram + ' ' + sound;
                    exec(speechCommand);
                }
            }
        } else {
            // Log to console and web if we are force to be quiet
            console.log(sound);
        }
    });
    // Send 'sound' to myself via Pushover
    if (personalData.pushover.USER !== "" && sound !== '' && sound !== undefined && sound !== null) {
        var p = new push({
            user: personalData.pushover.USER,
            token: personalData.pushover.TOKEN
        });
        var msg = {
            message: sound,
            sound: personalData.pushover.sound,
            priority: -1
        };
        p.send(msg); // Silent with no error reporting
    }
};
