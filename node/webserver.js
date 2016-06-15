var personalData = require('./personalData');
var webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
const Camera = require('./Camera');
var camera = new Camera('Camera', personalData.camera0name);
var robotModel = require('./robotModel');
const LaunchScript = require('./LaunchScript');
const tts = require('./tts');

const WayPoints = require('./WayPoints.js');
var wayPointEditor = new WayPoints();

const rosInterface = require('./rosInterface');
const fs = require('fs');
const express = require('express');
const spawn = require('child_process').spawn;
const bodyParser = require('body-parser');
const masterRelay = require('./MasterRelay');
const UsbRelay = require('./UsbRelayControl');
var usbRelay = new UsbRelay();

const updateMapList = require('./updateMapList');
updateMapList();

var app = express();
// For json encoded post requests, which I use:
app.use(bodyParser.json());
// Required for Twilio:
app.use(bodyParser.urlencoded({
    extended: true
}));

// All of my "static" web pages are in the public folder
app.use(express.static(__dirname + '/public'));
// The new experimental Angular 2 site is in the website folder ABOVE this one
app.use(express.static(__dirname + '/../website'));

var handleSemaphoreFiles = require('./handleSemaphoreFiles');
handleSemaphoreFiles.readSemaphoreFiles();

var saveMap = function (newMapName) {
    //TODO: mapDir should be defined less statically, no?
    var mapDir = process.env.HOME + '/.arlobot/rosmaps/';
    serverMapProcess = new LaunchScript({
        name: 'SaveMap',
        callback: updateMapList,
        ROScommand: 'rosrun map_server map_saver -f ' + mapDir + newMapName,
        scriptArguments: newMapName
    });
    //console.log(serverMapProcess.ROScommand);
    serverMapProcess.start();
};

var startLogStreamer = function () {
    var command = __dirname + '/../scripts/log-watcher.sh';
    var logStreamerProcess = spawn(command);
    logStreamerProcess.stdout.setEncoding('utf8');
    logStreamerProcess.stdout.on('data', function (data) {
        //console.log(data);
    });
    logStreamerProcess.stderr.setEncoding('utf8');
    logStreamerProcess.stderr.on('data', function (data) {
        //console.log(data);
    });
    logStreamerProcess.on('error', function (err) {
        //console.log(err);
    });
    logStreamerProcess.on('exit', function (code) {
        // Will catch multiple exit codes I think:
        if (code === 0) {
            webModelFunctions.scrollingStatusUpdate('Log streamer started');
            webModelFunctions.update('logStreamerRunning', true);
        } else {
            console.log('Log streamer failed with code: ' + code);
            webModelFunctions.update('logStreamerRunning', false);
        }
    });
    return logStreamerProcess;
};
var stopLogStreamer = function () {
    var command = '/usr/bin/pkill';
    var commandArgs = ['-f', 'log.io'];
    var process = spawn(command, commandArgs);
    process.stdout.setEncoding('utf8');
    process.stdout.on('data', function (data) {
        //console.log(data);
    });
    process.stderr.setEncoding('utf8');
    process.stderr.on('data', function (data) {
        //console.log(data);
    });
    process.on('error', function (err) {
        //console.log(err);
    });
    process.on('exit', function (code) {
        //console.log(code);
        webModelFunctions.scrollingStatusUpdate('Log streamer killed');
        webModelFunctions.update('logStreamerRunning', false);
    });
    return process;
};

// TODO: Shouldn't this be a behavior in along with Explore and Load Map?
// This is a good example of integrating a simple ROS function into the web menu,
// without having to redesign everything.
// Place the buttons for things like this into the "Behavior" section,
// and only show them when ROS is starting.
// Behaviors like these could also fall into "ramdon activities" when robot is "idle",
// but then I think that it would need to be in the behavior tree
var startColorFollower = function () {
    webModelFunctions.scrollingStatusUpdate('Starting Color Follower.');
    var command = __dirname + '/../scripts/object_follower.sh';
    var colorFollowerProcess = spawn(command);
    colorFollowerProcess.stdout.setEncoding('utf8');
    colorFollowerProcess.stdout.on('data', function (data) {
        if (data.indexOf('ROI messages detected. Starting follower...') > -1) {
            webModelFunctions.update('colorFollowerRunning', true);
            webModelFunctions.scrollingStatusUpdate('Color Follower has started.');
            webModelFunctions.behaviorStatusUpdate('Color Follower started.');
        }
        // console.log(data);
    });
    colorFollowerProcess.stderr.setEncoding('utf8');
    colorFollowerProcess.stderr.on('data', function (data) {
        console.log('colorFollower:', data);
    });
    colorFollowerProcess.on('error', function (err) {
        //console.log(err);
    });
    colorFollowerProcess.on('exit', function (code) {
        // Will catch multiple exit codes I think:
        if (code === 0) {
            webModelFunctions.scrollingStatusUpdate('Color Follower ended normally.');
        } else {
            console.log('Color Follower failed with code: ' + code);
        }
        webModelFunctions.update('colorFollowerRunning', false);
    });
    return colorFollowerProcess;
};
var stopColorFollower = function () {
    //pkill -f "roslaunch metatron_launchers object_follower.launch"
    var command = '/usr/bin/pkill';
    var commandArgs = ['-f', 'roslaunch metatron_launchers object_follower.launch'];
    var process = spawn(command, commandArgs);
    process.stdout.setEncoding('utf8');
    process.stdout.on('data', function (data) {
        //console.log(data);
    });
    process.stderr.setEncoding('utf8');
    process.stderr.on('data', function (data) {
        //console.log(data);
    });
    process.on('error', function (err) {
        //console.log(err);
    });
    process.on('exit', function (code) {
        //console.log(code);
    });
    return process;
};

// Kiosk button handler:
/* The 'kiosk' is a small web page/app, typically run on a smart phone,
 that is used for very basic robot settings,
 currently the primary functions are to force the robot to be
 quiet and to stop it. This is useful for allowing third parties in the
 vicinity of the robot to have some safety and convenience control
 over the robot, since it is not responsive to verbal commands. */
// TODO: Make this socket.io based, or at least help it update its own buttons!
app.post('/kioskBackEnd', function (req, res) {
    if (req.body.PLEASE) handleSemaphoreFiles.setSemaphoreFiles(req.body.PLEASE);
    var response = "{ \"QUIET\": " + webModel.beQuiet + ", \"STOP\": " + webModel.haltRobot + " }";
    res.send(response);
});

// Twilio SMS Receiver:
// TODO: This no longer works, we need to deal with  messages via the RobotMessageHandler! and remove the twilio code here.
app.post('/receivemessage', function (req, res) {
    // If you want a text response:
    //var twiml = '<?xml version="1.0" encoding="UTF-8" ?><Response><Message>Got it!</Message></Response>';
    // Otherwise, just tell Twilio we got it:
    var twiml = '<?xml version="1.0" encoding="UTF-8" ?><Response></Response>';
    res.send(twiml, {
        'Content-Type': 'text/xml'
    }, 200);
    console.log("Body: " + req.body.Body);
    console.log("From: " + req.body.From);
    webModelFunctions.scrollingStatusUpdate('Twilio:');
    webModelFunctions.scrollingStatusUpdate(req.body.Body);
    webModelFunctions.scrollingStatusUpdate('from: ' + req.body.From);
    // Make sure it is from ME! ;)
    if (req.body.From === '+13162087309') {
        if (req.body.Body.toLowerCase().indexOf('unplug') > -1) {
            webModelFunctions.update('unplugYourself', true);
            rosInterface.unplugRobot(true);
        }
    }
    // TODO: Take action based in incoming messages!
    // Probably in the behavior tree somewhere.
    // And/or pass some to speech/response system.
});

function start() {
    var webServer = app.listen(personalData.webServerPort);
    var io = require("socket.io").listen(webServer);

    webModelFunctions.emitter.on('change', function () {
        io.sockets.emit('webModel', webModel);
    });

    // Socket listeners
    io.on('connection', function (socket) {
        socket.emit('startup', webModel);
        var address = socket.request.connection.remoteAddress;
        console.log("Web connection from " + address);

        socket.on('setMap', function (data) {
            if (data) {
                if (data === 'Explore!') {
                    webModelFunctions.update('mapName', '');
                    webModelFunctions.update('autoExplore', true);
                } else if (webModel.mapList.indexOf(data)) {
                    webModelFunctions.update('autoExplore', false);
                    webModelFunctions.update('mapName', data);
                    wayPointEditor.updateWayPointList();
                }
            }
        });

        socket.on('clearMap', function () {
            if (!webModel.ROSisRunning) {
                webModelFunctions.update('autoExplore', false);
                webModelFunctions.update('mapName', '');
            }
        });

        socket.on('gotoWayPoint', function (data) {
            if (data) {
                if (webModel.wayPoints.indexOf(data) > -1) {
                    webModelFunctions.updateWayPointNavigator('wayPointName', data);
                    wayPointEditor.getWayPoint(data, function (response) {
                        robotModel.wayPointNavigator.destinaitonWaypoint = response;
                        webModelFunctions.updateWayPointNavigator('goToWaypoint', true);
                    });
                }
            }
        });

        // LocalMenu button handlers:
        socket.on('setWayPoint', function (data) {
            wayPointEditor.createWayPoint(data);
            setTimeout(wayPointEditor.updateWayPointList, 5000);
        });
        socket.on('tts', function (data) {
            tts(data);
        });
        socket.on('startROS', function () {
            if (webModel.logStreamerRunning) {
                stopLogStreamer();
            }
            webModelFunctions.update('ROSstart', true);
        });
        socket.on('stopROS', function () {
            webModelFunctions.update('ROSstart', false);
        });

        socket.on('haltRobot', function () {
            handleSemaphoreFiles.setSemaphoreFiles('stop');
        });
        socket.on('unHaltRobot', function () {
            handleSemaphoreFiles.setSemaphoreFiles('go');
        });
        socket.on('beQuiet', function () {
            handleSemaphoreFiles.setSemaphoreFiles('beQuiet');
        });
        socket.on('talk', function () {
            handleSemaphoreFiles.setSemaphoreFiles('talk');
        });
        socket.on('markDoorsClosed', function () {
            handleSemaphoreFiles.setSemaphoreFiles('markDoorsClosed');
        });

        socket.on('pauseAutoExplore', function () {
            webModelFunctions.update('pauseExplore', true);
        });
        socket.on('unPauseAutoExplore', function () {
            webModelFunctions.update('pauseExplore', false);
        });
        // unplugYourself
        socket.on('unplugYourself', function () {
            webModelFunctions.update('unplugYourself', true);
            rosInterface.unplugRobot(true);
        });
        socket.on('doNotUnplugYourself', function () {
            webModelFunctions.update('unplugYourself', false);
            rosInterface.unplugRobot(false);
        });

        // ROS Parameters
        socket.on('monitorAC', function () {
            rosInterface.setParam('monitorACconnection', true);
        });
        socket.on('ignoreAC', function () {
            rosInterface.setParam('monitorACconnection', false);
        });

        socket.on('monitorIR', function () {
            rosInterface.setParam('ignoreIRSensors', false);
        });
        socket.on('ignoreIR', function () {
            rosInterface.setParam('ignoreIRSensors', true);
        });

        socket.on('monitorCliff', function () {
            rosInterface.setParam('ignoreCliffSensors', false);
        });
        socket.on('ignoreCliff', function () {
            rosInterface.setParam('ignoreCliffSensors', true);
        });

        socket.on('monitorFloor', function () {
            rosInterface.setParam('ignoreFloorSensors', false);
        });
        socket.on('ignoreFloor', function () {
            rosInterface.setParam('ignoreFloorSensors', true);
        });

        socket.on('monitorProximity', function () {
            rosInterface.setParam('ignoreProximity', false);
        });
        socket.on('ignoreProximity', function () {
            rosInterface.setParam('ignoreProximity', true);
        });

        socket.on('saveMap', function (data) {
            console.log('Save map as: ' + data);
            saveMap(data);
        });
        socket.on('startLogStreamer', function () {
            startLogStreamer();
        });
        socket.on('stopLogStreamer', function () {
            stopLogStreamer();
        });
        socket.on('toggleLogStreamer', function () {
            if (webModel.logStreamerRunning) {
                stopLogStreamer();
            } else {
                startLogStreamer();
            }
        });
        socket.on('startColorFollower', function () {
            startColorFollower();
        });
        socket.on('stopColorFollower', function () {
            stopColorFollower();
        });
        socket.on('toggleDebug', function () {
            webModelFunctions.toggle('debugging');
        });
        socket.on('toggleCamera', function () {
            camera.toggle();
        });
        socket.on('toggleMasterRelay', function () {
            masterRelay('toggle');
        });
        socket.on('toggleRelay', function (data) {
            usbRelay.toggle(data);
        });
        socket.on('toggleRelayByName', function (data) {
            usbRelay.toggle(webModel.relays.find(x=> x.name === data)['number']);
        });
        socket.on('exit', function () {
            console.log('Shutdown requested from web interface!');
            webModelFunctions.scrollingStatusUpdate('Shutdown requested from web interface!');
            webModelFunctions.update('shutdownRequested', true);
        });
    });
}

exports.start = start;
