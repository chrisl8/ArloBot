var fs = require('fs');
var exec = require('child_process').exec;
var say = require('say');
var push = require('pushover-notifications');

module.exports = function(sound) {

    // This will not make any noise if the file
    // ~/.arlobot/status/bequiet
    // exists
    var beQuietFile = process.env.HOME + '/.arlobot/status/bequiet';
    fs.open(beQuietFile, 'r', function(err) {
        if (err) {

            // Get personal settings
            var personalDataFile = process.env.HOME + '/.arlobot/personalDataForBehavior.json';
            var personalData = JSON.parse(fs.readFileSync(personalDataFile, 'utf8'));

            // Set volume at max
            var setVolumeCommand = '/usr/bin/amixer set Master ' + personalData.speechVolumeLevelDefault + '% on';
            exec(setVolumeCommand);

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
                    say.speak(null, process.argv[2]);
                } else if (personalData.speechProgram) {
                    var speechCommand = personalData.speechProgram + ' ' + process.argv[2];
                    exec(speechCommand);
                }
            }
        }
    });
    // Send 'sound' to myself via Pushover
    // Load personal settings to get Pushover User and Token
    var personalDataFile = process.env.HOME + '/.arlobot/personalDataForBehavior.json';
    fs.readFile(personalDataFile, function(err, data) {
        if (!err) {
            var personalData = JSON.parse(data);
            if (personalData.pushover.USER !== "") {
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
        }
    });
};
