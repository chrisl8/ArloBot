var webModel = {
    ROSstart: false,
    ROSisRunning: false,
    pluggedIn: 'unknown',
    autoExplore: false,
    pauseExplore: false,
    beQuiet: false,
    haltRobot: false,
    basementDoorOpen: true,
    laptopFullyCharged: 'unknown',
    laptopBatteryPercentage: '???%',
    logStreamerRunning: false,
    status: 'Arlo behavior is not running.',
    mapList: ['Explore!'],
    mapName: ''
};

var processTracker = {
    logStreamerProcess: ''
};

// Set map list based on file names in the map folder
var getMapList = require('../getMapList');
var mapDir = process.env.HOME + '/.arlobot/rosmaps/';
getMapList(mapDir, function(err, data) {
    data.forEach(function(value) {
        webModel.mapList.push(value.replace('.yaml', ''));
    });
});

var fs = require('fs');
// Load personal settings not included in git repo
var personalDataFile = process.env.HOME + '/.arlobot/personalDataForBehavior.json';
var personalData = JSON.parse(fs.readFileSync(personalDataFile, 'utf8'));

var express = require('express');
var spawn = require('child_process').spawn;
var app = express();
var bodyParser = require('body-parser');
// For json encoded post requests, which I use:
app.use(bodyParser.json());
// Required for Twilio:
app.use(bodyParser.urlencoded({
    extended: true
}));
var mkdirp = require('mkdirp');
var exec = require('child_process').exec;

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
    //TODO: This does NOT create world writable folders. :(
    mkdirp(statusFolder, 0777, function(err) {
        if (err) {
            res.send("{\"STATUS\": \"ERROR\" }");
            console.log("Could not create " + statusFolder);
        } else {
            if (text === 'talk') {
                webModel.beQuiet = false;
                fs.unlink(quietFile);
            } else if (text === 'beQuiet') {
                webModel.beQuiet = true;
                fs.writeFile(quietFile, 'quiet\n');
            } else if (text === 'stop') {
                webModel.haltRobot = true;
                fs.writeFile(stopFile, 'STOP\n');
            } else if (text === 'go') {
                webModel.haltRobot = false;
                fs.unlink(stopFile);
            } else if (text === 'markBasementClosed') {
                fs.unlink(basementDoorFile);
                readSemaphoreFiles();
            }
        }
    });
};

var readSemaphoreFiles = function() {
    fs.readFile(stopFile, function(err) {
        if (err) webModel.haltRobot = false;
        else webModel.haltRobot = true;
    });
    fs.readFile(quietFile, function(err) {
        if (err) webModel.beQuiet = false;
        else webModel.beQuiet = true;
    });
    fs.readFile(basementDoorFile, 'utf8', function(err, data) {
        if (err) webModel.basementDoorOpen = false;
        else {
            // Alarm system puts the word 'GO' in the file when the door is closed.
            if (data === 'GO') webModel.basementDoorOpen = false;
            // We will consider anything else a 'STOP'
            else webModel.basementDoorOpen = true;
        }
    });
};

readSemaphoreFiles();

var saveMap = function(newMapName) {
    console.log(newMapName);
    var command = __dirname + '/../../scripts/save-map.sh';
    var saveMapProcess = spawn(command, newMapName);
    saveMapProcess.stdout.setEncoding('utf8');
    saveMapProcess.stdout.on('data', function(data) {
        console.log(data);
        //TODO: Parse this for a success code?
        if (data.indexOf('[master] killing on exit') > -1) {
            console.log('Save map process gave this data.');
        }
    });
    saveMapProcess.stderr.setEncoding('utf8');
    saveMapProcess.stderr.on('data', function(data) {
        console.log(data);
    });
    saveMapProcess.on('error', function(err) {
        console.log(err);
    });
    saveMapProcess.on('exit', function(code) {
        // Will catch multiple exit codes I think:
        if (code === 0) {
            console.log('Save map exited with code 0.');
        } else {
            console.log('Save map failed with code: ' + code);
        }
    });
};

var startLogStreamer = function() {
    var command = __dirname + '/../../scripts/log-watcher.sh';
    var logStreamerProcess = spawn(command);
    logStreamerProcess.stdout.setEncoding('utf8');
    logStreamerProcess.stdout.on('data', function(data) {
        console.log(data);
    });
    logStreamerProcess.stderr.setEncoding('utf8');
    logStreamerProcess.stderr.on('data', function(data) {
        console.log(data);
    });
    logStreamerProcess.on('error', function(err) {
        console.log(err);
    });
    logStreamerProcess.on('exit', function(code) {
        // Will catch multiple exit codes I think:
        if (code === 0) {
            console.log('Log streamer exited with code 0.');
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
        console.log(data);
    });
    process.stderr.setEncoding('utf8');
    process.stderr.on('data', function(data) {
        console.log(data);
    });
    process.on('error', function(err) {
        console.log(err);
    });
    process.on('exit', function(code) {
        console.log(code);
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
    var webServer = app.listen(8080);
    var ngrok = require('ngrok');
    var io = require("socket.io").listen(webServer);
    ngrok.connect({
        authtoken: personalData.ngrok.authtoken,
        subdomain: personalData.ngrok.subdomain,
        port: personalData.ngrok.port
    }, function(err, url) {
        console.log('ngrok URL: ' + url);
        if (err != null) {
            console.log(err);
        }
    });

    exports.updateWebModel = function(variable, value) {
        if ((webModel[variable] != value)) {
            webModel[variable] = value;
            io.sockets.emit('webModel', webModel);
        }
    };

    // Socket listeners
    io.on('connection', function(socket) {
        socket.emit('startup', webModel);
        console.log('Connection!');

        socket.on('setMap', function(data) {
            console.log('Set map: ' + data);
            if (data != null) {
                if (data === 'Explore!') {
                    webModel.mapName = '';
                    webModel.autoExplore = true;
                } else if (webModel.mapList.indexOf(data)) {
                    webModel.autoExplore = false;
                    webModel.mapName = data;
                }
            }
            io.sockets.emit('webModel', webModel);
        });

        // LocalMenu button handlers:
        socket.on('startROS', function() {
            webModel.ROSstart = true;
            io.sockets.emit('webModel', webModel);
        });
        socket.on('stopROS', function() {
            webModel.ROSstart = false;
            io.sockets.emit('webModel', webModel);
        });
        socket.on('haltRobot', function() {
            webModel.haltRobot = true;
            setSemaphoreFiles('stop');
            io.sockets.emit('webModel', webModel);
        });
        socket.on('unHaltRobot', function() {
            webModel.haltRobot = false;
            setSemaphoreFiles('go');
            io.sockets.emit('webModel', webModel);
        });
        socket.on('beQuiet', function() {
            webModel.beQuiet = true;
            setSemaphoreFiles('beQuiet');
            io.sockets.emit('webModel', webModel);
        });
        socket.on('talk', function() {
            webModel.beQuiet = false;
            setSemaphoreFiles('talk');
            io.sockets.emit('webModel', webModel);
        });
        socket.on('markBasementClosed', function() {
            setSemaphoreFiles('markBasementClosed');
            webModel.basementDoorOpen = false;
            io.sockets.emit('webModel', webModel);
        });
        socket.on('pauseAutoExplore', function() {
            webModel.pauseExplore = true;
            io.sockets.emit('webModel', webModel);
        });
        socket.on('unPauseAutoExplore', function() {
            webModel.pauseExplore = false;
            io.sockets.emit('webModel', webModel);
        });
        socket.on('saveMap', function(data) {
            console.log('Save map as: ' + data);
            saveMap(data);
        });
        socket.on('startLogStreamer', function() {
            startLogStreamer();
            webModel.logStreamerRunning = true;
            io.sockets.emit('webModel', webModel);
        });
        socket.on('stopLogStreamer', function() {
            stopLogStreamer();
            webModel.logStreamerRunning = false;
            io.sockets.emit('webModel', webModel);
        });
    });

    return io;
}

exports.start = start;
exports.webModel = webModel;

//var server = app.listen(8080, function() {
//var host = server.address().address;
//var port = server.address().port;
//console.log('Example app listening at http://%s:%s', host, port);
//});
