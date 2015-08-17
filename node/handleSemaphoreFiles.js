// This code places 'semaphore' files into the file system
// that the ROS Python code and other node apps
// watch and respond to.
var webModel = require('./webModel');
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
            if (err) webModel[value] = false;
            else webModel[value] = true;
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
            webModel.doorsOpen = true;
        } else {
            if (files.length > 0) {
                webModel.doorsOpen = true;
            } else {
                webModel.doorsOpen = false;
            }
        }
    });
};

exports.setSemaphoreFiles = setSemaphoreFiles;
exports.readSemaphoreFiles = readSemaphoreFiles;
