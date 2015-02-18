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

        var requestedSound = process.argv[2];

        // This script can accept text to speak,
        // or .wav files to play.
        // We rely stricly on the extension to
        // decide what it is!
        var possibleExtension = requestedSound.slice(-4).toLowerCase();
        if (possibleExtension === '.wav') {
            exec('/usr/bin/aplay -q ' + requestedSound);
        } else {
            // https://github.com/marak/say.js/
            var say = require('say');
            // no callback, fire and forget
            say.speak(null, process.argv[2]);
        }
    }
});
