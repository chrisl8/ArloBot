// This code places 'semaphore' files into the file system
// that the ROS Python code and other node apps
// watch and respond to.
// This is called by the Poll tree function,
// so if index.js or the Behavior Tree hang up, this won't happen,
// but then the robot won't be good for much at that point anyway.
// I'm just pointing out that this doesn't actively monitor anything,
// it is called in a polling loop by index.js
var webModel = require('./webModel');
var webModelFunctions = require('./webModelFunctions');
var fs = require('fs');
var mkdirp = require('mkdirp');

var personalDataFolder = process.env.HOME + '/.arlobot/';
var statusFolder = personalDataFolder + 'status/';
var quietFile = statusFolder + 'bequiet';
var stopFile = statusFolder + 'STOP';
var doorFileFolder = statusFolder + '/doors';
// Note this will only work if we do not KNOW what map we are on.
var doorFile = statusFolder + '/doors/unknown-door';

var setSemaphoreFiles = function(text) {
    //NOTE: This does NOT create world writable folders. :(
    // But the setup program should have already created it for us anyway.
    mkdirp(statusFolder, 0777, function(err) {
        if (err) {
            res.send("{\"STATUS\": \"ERROR\" }");
            console.log("Could not create " + statusFolder);
        } else {
            if (text === 'talk') {
                webModelFunctions.update('beQuiet', false);
                fs.unlink(quietFile, readSemaphoreFiles);
            } else if (text === 'beQuiet') {
                webModelFunctions.update('beQuiet', true);
                fs.writeFile(quietFile, 'quiet\n');
            } else if (text === 'go') {
                webModelFunctions.update('haltRobot', false);
                fs.unlink(stopFile, readSemaphoreFiles);
            } else if (text === 'stop') {
                webModelFunctions.update('haltRobot', true);
                fs.writeFile(stopFile, 'STOP\n');
            } else if (text === 'markDoorsClosed') {
                // Wipe out ALL door files if asked!
                // The "right" way is to test the doors,
                // but that will leave files that will prevent
                // exploring, and will keep the PINK warning
                // button on even if robot will go.
                fs.readdir(doorFileFolder, function(err, files) {
                    files.forEach(function(file) {
                        fs.unlink(doorFileFolder + '/' + file, readSemaphoreFiles);
                    });
                });
            } else if (text === 'markDoorsOpen') {
                fs.writeFile(doorFile, 'STOP\n');
            }
        }
    });
};

var readSemaphoreFiles = function() {

    var checkFileAndSetValue = function(file, value) {
        fs.readFile(file, 'utf8', function(err, data) {
            if (err){
                webModelFunctions.update(value, false);
            } else {
                webModelFunctions.update(value, true);
            }
        });
    };

    checkFileAndSetValue(stopFile, 'haltRobot');
    checkFileAndSetValue(quietFile, 'beQuiet');

    // Check door files
    // TODO: How can we tell if the folder only has files
    // in it for the wrong map?
    fs.readdir(doorFileFolder, function(err, files) {
        var newValue;
        if (err) {
            console.log('Door folder problem: ' + err);
            // True on error for safety.
            webModelFunctions.update('doorsOpen', true);
        } else {
            if (files.length > 0) {
                webModelFunctions.update('doorsOpen', true);
            } else {
                webModelFunctions.update('doorsOpen', false);
            }
        }
    });
};

exports.setSemaphoreFiles = setSemaphoreFiles;
exports.readSemaphoreFiles = readSemaphoreFiles;
