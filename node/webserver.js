var webModel = require('./webModel');
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
var mkdirp = require('mkdirp');
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

// Load personal settings not included in git repo
var personalDataFile = process.env.HOME + '/.arlobot/personalDataForBehavior.json';
var personalData = JSON.parse(fs.readFileSync(personalDataFile, 'utf8'));

var app = express();
// For json encoded post requests, which I use:
app.use(bodyParser.json());
// Required for Twilio:
app.use(bodyParser.urlencoded({
    extended: true
}));

// All of my static web pages are in the public folder
app.use(express.static(__dirname + '/public'));

// This code places 'semaphore' files into the file system
// that the ROS Python code and other node apps
// watch and respond to.
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

readSemaphoreFiles();

var saveMap = function(newMapName) {
    serverMapProcess = new LaunchScript({
        name: 'SaveMap',
        callback: updateMapList,
        ROScommand: 'rosrun map_server map_saver -f ' + mapDir + newMapName,
        scriptArguments: newMapName
    });
    console.log(serverMapProcess.ROScommand);
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
            webModel.scrollingStatus = 'Log streamer exited with code 0.<br/>' + webModel.scrollingStatus;
            //console.log('Log streamer exited with code 0.');
        } else {
            console.log('Log streamer failed with code: ' + code);
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
app.post('/kioskBackEnd', function(req, res) {
    if (req.body.PLEASE) setSemaphoreFiles(req.body.PLEASE);
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
    /*
    I'm not going to use ROS to parse these anymore,
    I'll parse them in node and act on them in node,
    or direct ROS directly from node.
    There is no reason for the text parsing code to be in ROS/Python.
    if (req.body.Body === 's')
        var command = './runROSthings.sh ' + req.body.Body + ' ' + req.body.From;
    var ROScommand = exec(command);
    ROScommand.stdout.on('data', function(data) {
        console.log('stdout: ' + data);
    });

    ROScommand.stderr.on('data', function(data) {
        console.log('stderr: ' + data);
    });

    ROScommand.on('close', function(code) {
        console.log('child process exited with code ' + code);
    });
    ROScommand.on('error', function(err) {
        console.log('child process error' + err);
    });
*/
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
            webModel.haltRobot = true;
            setSemaphoreFiles('stop');
        });
        socket.on('unHaltRobot', function() {
            webModel.haltRobot = false;
            setSemaphoreFiles('go');
        });
        socket.on('beQuiet', function() {
            webModel.beQuiet = true;
            setSemaphoreFiles('beQuiet');
        });
        socket.on('talk', function() {
            webModel.beQuiet = false;
            setSemaphoreFiles('talk');
        });
        socket.on('markBasementClosed', function() {
            setSemaphoreFiles('markBasementClosed');
            webModel.basementDoorOpen = false;
        });
        socket.on('pauseAutoExplore', function() {
            webModel.pauseExplore = true;
        });
        socket.on('unPauseAutoExplore', function() {
            webModel.pauseExplore = false;
        });
        // TODO: This should also affect the ROS safety setting parameter.
        socket.on('monitorAC', function() {
            webModel.ignorePluggedIn = false;
        });
        socket.on('ignoreAC', function() {
            webModel.ignorePluggedIn = true;
        });

        // ROS Parameters
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
            webModel.logStreamerRunning = true;
        });
        socket.on('stopLogStreamer', function() {
            stopLogStreamer();
            webModel.logStreamerRunning = false;
        });
        socket.on('exit', function() {
            webModel.shutdownRequested = true;
        });
    });
}

exports.start = start;
