const exec = require('child_process').exec;

const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');

function killROS(exitWhenDone) {
    'use strict';
    const command = __dirname + '/../scripts/kill_ros.sh';
    // It is rather catastrophic if this repeats!
    if (!webModel.killRosHasRun) {
        webModelFunctions.update('killRosHasRun', true);
        webModelFunctions.update('ROSstart', false);
        // ROS drops all knowledge when restarted,
        // So make it clear to the user that he needs to pick a map again.
        webModelFunctions.update('mapName', '');
        webModelFunctions.update('autoExplore', false);
        webModelFunctions.scrollingStatusUpdate("Running kill_ros.sh . . .");
        // Logging to console too, because feedback on shutdown is nice.
        console.log("Running kill_ros.sh . . .");
        // and then also run the kill ROS command:
        const shutdownCommand = exec(command);
        shutdownCommand.stdout.on('data', function (data) {
            webModelFunctions.scrollingStatusUpdate('Shutdown: ' + data);
            console.log('Shutdown:' + data.toString().replace(/[\n\r]/g, ""));
        });

        shutdownCommand.stderr.on('data', function (data) {
            webModelFunctions.scrollingStatusUpdate('Shutdown: ' + data);
            console.log('Shutdown:' + data.toString().replace(/[\n\r]/g, ""));
        });

        shutdownCommand.on('close', function (code) {
            webModelFunctions.scrollingStatusUpdate('kill_ros.sh closed with code ' + code);
            console.log('kill_ros.sh closed with code ' + code);
            if (exitWhenDone) {
                process.exit();
            } else {
                webModel.killRosHasRun = false;
                webModelFunctions.update('ROSisRunning', false);
            }
        });
        shutdownCommand.on('error', function (err) {
            webModelFunctions.scrollingStatusUpdate('Shutdown process error' + err);
        });
    }
}

module.exports = killROS;
