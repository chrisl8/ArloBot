// This code places 'semaphore' files into the file system
// that the ROS Python code and other node apps
// watch and respond to.
var webModel = require('./webModel');
var fs = require('fs');
var mkdirp = require('mkdirp');

var personalDataFolder = process.env.HOME + '/.arlobot/';
var statusFolder = personalDataFolder + 'status/';
var quietFile = statusFolder + 'bequiet';
var stopFile = statusFolder + 'webStopRequested';
var basementDoorFile = statusFolder + 'room-MainFloorHome';
var setSemaphoreFiles = function(text) {
    //NOTE: This does NOT create world writable folders. :(
    // But the setup program should have already created it for us anyway.
    mkdirp(statusFolder, 0777, function(err) {
        if (err) {
            res.send("{\"STATUS\": \"ERROR\" }");
            console.log("Could not create " + statusFolder);
        } else {
            if (text === 'talk') {
                webModel.beQuiet = false;
                fs.unlink(quietFile, readSemaphoreFiles);
            } else if (text === 'beQuiet') {
                webModel.beQuiet = true;
                fs.writeFile(quietFile, 'quiet\n');
            } else if (text === 'go') {
                webModel.haltRobot = false;
                fs.unlink(stopFile, readSemaphoreFiles);
            } else if (text === 'stop') {
                webModel.haltRobot = true;
                fs.writeFile(stopFile, 'STOP\n');
            } else if (text === 'markBasementClosed') {
                fs.unlink(basementDoorFile, readSemaphoreFiles);
            }
        }
    });
};

var readSemaphoreFiles = function() {

    var checkFileAndSetValue = function(file, value) {
        fs.readFile(file, 'utf8', function(err, data) {
            var oldValue = webModel[value];
            if (err) webModel[value] = false;
            else {
                /* For basement door open,
                alarm system puts the word 'GO'
                into the file instead of deleting it.
                */
                if (data.indexOf('GO') > -1) webModel[value] = false;
                // We will consider anything else a 'STOP'
                else webModel[value] = true;
            }
        });
    };

    checkFileAndSetValue(stopFile, 'haltRobot');
    checkFileAndSetValue(quietFile, 'beQuiet');
    checkFileAndSetValue(basementDoorFile, 'basementDoorOpen');

};

exports.setSemaphoreFiles = setSemaphoreFiles;
exports.readSemaphoreFiles = readSemaphoreFiles;
