var personalData = require('./personalData');
var webModel = require('./webModel');
var webModelFunctions = require('./webModelFunctions');
var O = require('observed');
var webModelWatcher = O(webModel);
var LaunchScript = require('./LaunchScript');

var WayPoints = require('./WayPoints.js');
var wayPointEditor = new WayPoints();

var rosInterface = require('./rosInterface');
var getMapList = require('./getMapList');
var fs = require('fs');
var express = require('express');
var spawn = require('child_process').spawn;
var bodyParser = require('body-parser');
var exec = require('child_process').exec;
var ngrok = require('ngrok');

var processTracker = {
    logStreamerProcess: ''
};

// Set map list based on file names in the map folder
var mapDir = process.env.HOME + '/.arlobot/rosmaps/';
var updateMapList = function() {
    webModel.mapList = ['Explore!'];
    getMapList(mapDir, function(err, data) {
        data.forEach(function(value) {
            webModel.mapList.push(value.replace('.yaml', ''));
        });
    });
};

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

var handleSemaphoreFiles = require('./handleSemaphoreFiles');
handleSemaphoreFiles.readSemaphoreFiles();

var saveMap = function(newMapName) {
    serverMapProcess = new LaunchScript({
        name: 'SaveMap',
        callback: updateMapList,
        ROScommand: 'rosrun map_server map_saver -f ' + mapDir + newMapName,
        scriptArguments: newMapName
    });
    //console.log(serverMapProcess.ROScommand);
    serverMapProcess.start();
};

var startLogStreamer = function() {
    var command = __dirname + '/../scripts/log-watcher.sh';
    var logStreamerProcess = spawn(command);
    logStreamerProcess.stdout.setEncoding('utf8');
    logStreamerProcess.stdout.on('data', function(data) {
        //console.log(data);
    });
    logStreamerProcess.stderr.setEncoding('utf8');
    logStreamerProcess.stderr.on('data', function(data) {
        //console.log(data);
    });
    logStreamerProcess.on('error', function(err) {
        //console.log(err);
    });
    logStreamerProcess.on('exit', function(code) {
        // Will catch multiple exit codes I think:
        if (code === 0) {
            webModelFunctions.scrollingStatusUpdate('Log streamer started');
            webModel.logStreamerRunning = true;
        } else {
            console.log('Log streamer failed with code: ' + code);
            webModel.logStreamerRunning = false;
        }
    });
    return logStreamerProcess;
};
var stopLogStreamer = function() {
    var command = '/usr/bin/pkill';
    var commandArgs = ['-f', 'log.io'];
    var process = spawn(command, commandArgs);
    process.stdout.setEncoding('utf8');
    process.stdout.on('data', function(data) {
        //console.log(data);
    });
    process.stderr.setEncoding('utf8');
    process.stderr.on('data', function(data) {
        //console.log(data);
    });
    process.on('error', function(err) {
        //console.log(err);
    });
    process.on('exit', function(code) {
        //console.log(code);
        webModelFunctions.scrollingStatusUpdate('Log streamer killed');
        webModel.logStreamerRunning = false;
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
app.post('/kioskBackEnd', function(req, res) {
    if (req.body.PLEASE) handleSemaphoreFiles.setSemaphoreFiles(req.body.PLEASE);
    var response = "{ \"QUIET\": " + webModel.beQuiet + ", \"STOP\": " + webModel.haltRobot + " }";
    res.send(response);
});

// Twilio SMS Receiver:
app.post('/receivemessage', function(req, res) {
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
            webModel.unplugYourself = true;
        }
    }
    // TODO: Take action based in incoming messages!
    // Probably in the behavior tree somewhere.
    // And/or pass some to speech/response system.
});

function start() {
    var webServer = app.listen(personalData.webServerPort);
    var io = require("socket.io").listen(webServer);
    ngrok.connect({
        authtoken: personalData.ngrok.authtoken,
        subdomain: personalData.ngrok.subdomain,
        port: personalData.ngrok.port
    }, function(err, url) {
        // NOTE: If you need this for Twilio:
        //console.log('ngrok URL: ' + url);
        if (err) {
            console.log(err);
        }
    });

    webModelWatcher.on('change', function() {
        io.sockets.emit('webModel', webModel);
    });

    // Socket listeners
    io.on('connection', function(socket) {
        socket.emit('startup', webModel);
        var address = socket.request.connection.remoteAddress;
        console.log("Web connection from " + address);

        socket.on('setMap', function(data) {
            if (data) {
                if (data === 'Explore!') {
                    webModel.mapName = '';
                    webModel.autoExplore = true;
                } else if (webModel.mapList.indexOf(data)) {
                    webModel.autoExplore = false;
                    webModel.mapName = data;
                    wayPointEditor.updateWayPointList();
                }
            }
        });

        socket.on('gotoWayPoint', function(data) {
            if (data) {
                if (webModel.wayPoints.indexOf(data) > -1) {
                    wayPointEditor.getWayPoint(data, function(response) {
                        var goToMapPositionProcess = new LaunchScript({
                            debugging: true,
                            name: 'GoToWaypoint',
                            ROScommand: 'unbuffer rosservice call /arlobot_goto/go_to_goal "' + response + '"'
                        });
                        goToMapPositionProcess.start();
                    });
                }
            }
        });

        // LocalMenu button handlers:
        socket.on('setWayPoint', function(data) {
            wayPointEditor.createWayPoint(data);
            setTimeout(wayPointEditor.updateWayPointList, 5000);
        });
        socket.on('startROS', function() {
            webModel.ROSstart = true;
        });
        socket.on('stopROS', function() {
            webModel.ROSstart = false;
        });

        socket.on('haltRobot', function() {
            handleSemaphoreFiles.setSemaphoreFiles('stop');
        });
        socket.on('unHaltRobot', function() {
            handleSemaphoreFiles.setSemaphoreFiles('go');
        });
        socket.on('beQuiet', function() {
            handleSemaphoreFiles.setSemaphoreFiles('beQuiet');
        });
        socket.on('talk', function() {
            handleSemaphoreFiles.setSemaphoreFiles('talk');
        });
        socket.on('markDoorsClosed', function() {
            handleSemaphoreFiles.setSemaphoreFiles('markDoorsClosed');
        });

        socket.on('pauseAutoExplore', function() {
            webModel.pauseExplore = true;
        });
        socket.on('unPauseAutoExplore', function() {
            webModel.pauseExplore = false;
        });

        // ROS Parameters
        socket.on('monitorAC', function() {
            rosInterface.setParam('monitorACconnection', true);
        });
        socket.on('ignoreAC', function() {
            rosInterface.setParam('monitorACconnection', false);
        });

        socket.on('monitorIR', function() {
            rosInterface.setParam('ignoreIRSensors', false);
        });
        socket.on('ignoreIR', function() {
            rosInterface.setParam('ignoreIRSensors', true);
        });

        socket.on('monitorCliff', function() {
            rosInterface.setParam('ignoreCliffSensors', false);
        });
        socket.on('ignoreCliff', function() {
            rosInterface.setParam('ignoreCliffSensors', true);
        });

        socket.on('monitorProximity', function() {
            rosInterface.setParam('ignoreProximity', false);
        });
        socket.on('ignoreProximity', function() {
            rosInterface.setParam('ignoreProximity', true);
        });

        socket.on('saveMap', function(data) {
            console.log('Save map as: ' + data);
            saveMap(data);
        });
        socket.on('startLogStreamer', function() {
            startLogStreamer();
        });
        socket.on('stopLogStreamer', function() {
            stopLogStreamer();
        });
        socket.on('exit', function() {
            webModel.shutdownRequested = true;
        });
    });
}

exports.start = start;
