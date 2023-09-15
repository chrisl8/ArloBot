const spawn = require('child_process').spawn;
const Push = require('pushover-notifications');
const fs = require('fs');
const { promisify } = require('util');

const readFile = promisify(fs.readFile);
const personalData = require('./personalData');
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
const myCroft = require('./MyCroft');
const robotModel = require('./robotModel');

async function tts(sound) {
  // It is fine to call this without 'await', since most of the time you don't want to wait for it to speak before
  // moving on with your programing.

  const checkFileAndSetValue = async (file, value) => {
    try {
      await readFile(file, 'utf8');
      webModelFunctions.update(value, true);
    } catch (e) {
      webModelFunctions.update(value, false);
    }
  };
  if (!webModel.semaphoreFilesRead) {
    // This will not make any noise if the file
    // ~/.arlobot/status/bequiet
    // exists

    // This section is for running from the command line.
    const personalDataFolder = `${process.env.HOME}/.arlobot/`;
    const statusFolder = `${personalDataFolder}status/`;
    const quietFile = `${statusFolder}bequiet`;
    await checkFileAndSetValue(quietFile, 'beQuiet');
  }

  if (!sound) {
    console.error('I was asked to say nothing.');
  } else if (webModel.beQuiet) {
    if (personalData.useMyCroft) {
      webModelFunctions.update(
        'myCroftSaid',
        'I cannot reply, because I was asked to be quiet.',
      );
    }
    // Log to console
    console.log(sound);
  } else {
    // Set volume at max
    if (!robotModel.volumeHasBeenSet) {
      // We don't wait for this, so the first speech may be quiet, but this isn't worth waiting for.
      spawn(`${__dirname}/../scripts/set_MasterVolume.sh`, [
        personalData.speechVolumeLevelDefault,
      ]);
      robotModel.volumeHasBeenSet = true;
    }

    // This script can accept text to speak,
    // or .wav files to play.
    // We rely strictly on the extension to
    // decide what it is!
    const possibleExtension = sound.slice(-4).toLowerCase();
    if (possibleExtension === '.wav') {
      const soundFile = sound.replace('~', process.env.HOME);
      /*
       $XDG_RUNTIME_DIR='/run/user/1000' is required for aplay to work,
       it is set when you log in, but never from cron or any automatic startup,
       so this fixes that.
       */
      const child = spawn('/usr/bin/aplay', ['-q', soundFile], {
        env: { ...process.env, XDG_RUNTIME_DIR: '/run/user/1000' },
      });
      let dataHolder = '';
      child.stdout.on('data', (data) => {
        if (data !== '') {
          dataHolder += data;
        }
      });

      child.stderr.on('data', (data) => {
        if (data !== '') {
          dataHolder += data;
        }
      });
      child.on('close', (code) => {
        if (code > 0) {
          console.error(`aplay closed with code: ${code}`);
        }
        if (dataHolder !== '') {
          // It should never return anything, so any return is an error.
          console.error(dataHolder);
        }
      });
    } else if (personalData.useMyCroft) {
      myCroft.sayText(sound);
    } else if (personalData.speechProgram) {
      spawn(personalData.speechProgram, [sound]);
    }
  }
  // Send 'sound' to myself via Pushover
  // TODO: Replace this with call to pushMe.js
  if (
    personalData.pushover.USER !== '' &&
    sound !== '' &&
    sound !== undefined &&
    sound !== null
  ) {
    const p = new Push({
      user: personalData.pushover.USER,
      token: personalData.pushover.TOKEN,
    });
    const msg = {
      message: sound,
      sound: personalData.pushover.sound,
      priority: -1,
    };
    p.send(msg); // Silent with no error reporting
  }
}

module.exports = tts;

if (require.main === module) {
  if (process.argv.length < 3) {
    console.log('You must provide text for the message, like this:');
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
