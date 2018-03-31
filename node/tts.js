const personalData = require('./personalData');
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
const handleSemaphoreFiles = require('./handleSemaphoreFiles');
const exec = require('child_process').exec;
const say = require('say');
const push = require('pushover-notifications');
const myCroft = require('./MyCroft');

async function tts(sound) {
    // It is fine to call this without 'await', since most of the time you don't want to wait for it to speak before
    // moving on with your programing.

    if (!webModel.semaphoreFilesRead) {
        // This will not make any noise if the file
        // ~/.arlobot/status/bequiet
        // exists
        await handleSemaphoreFiles.readSemaphoreFiles();
    }

    if (webModel.beQuiet) {
        if (personalData.useMyCroft) {
            webModelFunctions.update('myCroftSaid', 'I cannot reply, because I was asked to be quiet.');
        }
        // Log to console
        console.log(sound);
    } else {

        // Set volume at max
        const setVolumeCommand = '/usr/bin/amixer set Master ' + personalData.speechVolumeLevelDefault + '% on';
        // We don't wait for this, so the first speech may be quiet, but this isn't worth waiting for.
        exec(setVolumeCommand);

        // This script can accept text to speak,
        // or .wav files to play.
        // We rely strictly on the extension to
        // decide what it is!
        const possibleExtension = sound.slice(-4).toLowerCase();
        if (possibleExtension === '.wav') {
            exec('/usr/bin/mplayer -quiet ' + sound + ' > /dev/null 2>&1');
        } else {
            if (personalData.useMyCroft) {
                myCroft.sayText(sound);
            } else if (personalData.speechProgram === 'nodeSay') {
                // https://github.com/marak/say.js/
                // no callback, fire and forget
                say.speak(sound);
            } else if (personalData.speechProgram) {
                exec(`${personalData.speechProgram} "${sound}"`);
            }
        }
    }
    // Send 'sound' to myself via Pushover
    if (personalData.pushover.USER !== "" && sound !== '' && sound !== undefined && sound !== null) {
        const p = new push({
            user: personalData.pushover.USER,
            token: personalData.pushover.TOKEN
        });
        const msg = {
            message: sound,
            sound: personalData.pushover.sound,
            priority: -1
        };
        p.send(msg); // Silent with no error reporting
    }
}

module.exports = tts;

if (require.main === module) {
    if (process.argv.length < 3) {
        console.log("You must provide text for the message, like this:");
        console.log(`node tts.js "test message"`);
        process.exit();
    }
    try {
        tts(process.argv[2]);
    } catch (e) {
        console.error('TTS Error:');
        console.error(e);
    }
}
