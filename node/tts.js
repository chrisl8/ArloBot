module.exports = function(sound) {

    // This will not make any noise if the file
    // ~/.arlobot/status/bequiet
    // exists
    var beQuietFile = process.env.HOME + '/.arlobot/status/bequiet';
    var fs = require('fs');
    fs.open(beQuietFile, 'r', function(err) {
        if (err) {

            var exec = require('child_process').exec;
            // Set volume at max
            exec('/usr/bin/amixer -D pulse sset Master 100% on');

            // This script can accept text to speak,
            // or .wav files to play.
            // We rely stricly on the extension to
            // decide what it is!
            var possibleExtension = sound.slice(-4).toLowerCase();
            if (possibleExtension === '.wav') {
                exec('/usr/bin/aplay -q ' + sound);
            } else {
                // https://github.com/marak/say.js/
                var say = require('say');
                // no callback, fire and forget
                say.speak(null, process.argv[2]);
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
                var push = require('pushover-notifications');
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
