var webModel = require('./webModel');
var webModelFunctions = require('./webModelFunctions');
// TODO: If this bug is every fixed:
// https://github.com/RobotWebTools/roslibjs/issues/160
// Stop using my fork of roslib.
var ROSLIB = require('roslib');

// Copied from arloweb.js
var connectedToROS = false, // Track my opinion of the connection
    connectRequested = false, // For when we asked and are waiting patiently.
    pleaseWait = false, // Display Please Wait on the Connect button
    ros, // Empty global for actual connection.
    cmdVel, // Empty global for actual topic
    wakeScreen, // Empty global for actual topic
    toggleCamera, // Empty global for actual topic
    toggleRelay, // Empty global for actual topic
    sendTextToSpeak, // Empty global for actual topic
    shortDelay = 1000,
    longDelay = 3000,
    camera1On = false, // For tracking Camera status
    camera2On = false, // For tracking Camera status
    upperLightOn = false, // For tracking LED light bars
    lowerLightOn = false, // For tracking LED light bars
    UpperLightRowRelay = -1,
    UpperLightRowName = "TopLightRow", // Found in arlobot_usbrelay/param/usbrelay.yaml
    LowerLightRowRelay = -1,
    LowerLightRowName = "BottomLightRow", // Found in arlobot_usbrelay/param/usbrelay.yaml
    RightMotorRelay = -1,
    RightMotorName = "RightMotor", // Found in arlobot_usbrelay/param/usbrelay.yaml
    LeftMotorRelay = -1,
    LeftMotorName = "LeftMotor"; // Found in arlobot_usbrelay/param/usbrelay.yaml

// Copied from arloweb.js
// Be sure to set url to point to localhost,
// and change any references to web objects with console.log (i.e. setActionField)

// Define a list of ROS Parameters to monitor
// NOTE: Add an instance to webModel if you want this sent to the web app!
var rosParameters = {
        ignoreCliffSensors: {
            param: null,
            label: 'ignoreCliffSensors',
            path: '/arlobot/ignoreCliffSensors'
        },
        ignoreProximity: {
            param: null,
            label: 'ignoreProximity',
            path: '/arlobot/ignoreProximity'
        },
        ignoreIRSensors: {
            param: null,
            label: 'ignoreIRSensors',
            path: '/arlobot/ignoreIRSensors'
        },
        monitorACconnection: {
            param: null,
            label: 'monitorACconnection',
            path: '/arlobot/monitorACconnection'
        }
    };

var talkToROS = function() {
    // If you wanted to dump ALL params:
    //ros.getParams(function(params) {
    //    console.log('ROSLIB Params:');
    //    console.log(params);
    //});

    for (var prop in rosParameters) {
        rosParameters[prop].param = new ROSLIB.Param({
            ros: ros,
            name: rosParameters[prop].path
        });
    }

    pollParams();
};

var pollParams = function() {
    function checkParameter(prop) {
        rosParameters[prop].param.get(function(value) {
            //console.log(rosParameters[prop].label + ': ' + value);
            // Assign state to webModel object for view by web page.
            if (webModel.rosParameters.hasOwnProperty(prop)) {
                webModel.rosParameters[prop] = value;
                //console.log('For web: ' + webModel.rosParameters[prop]);
            }
        });
    }

    for (var prop in rosParameters) {
        if (rosParameters.hasOwnProperty(prop)) {
            checkParameter(prop);
        }
    }

    setTimeout(pollParams, longDelay);
};

var setParam = function(paramLabel, value) {
    if (rosParameters.hasOwnProperty(paramLabel)) {
        if (rosParameters[paramLabel].param) {
            rosParameters[paramLabel].param.set(value);
        }
    }
};

var pollROS = function() {
    // console.log('ROSLIB pollROS run');
    connectedToROS = false;

    ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090',
        // This eliminates a warning about utf8:
        encoding: 'ascii'
    });

    ros.on('connection', function() {
        webModelFunctions.scrollingStatusUpdate('ROSLIB Websocket connected.');
        //connectRequested = true;
        //updateConnectedButton();
        //checkROSServices();
        setTimeout(talkToROS, longDelay);
    });

    ros.on('error', function(error) {
        //console.log('Error connecting to websocket server: ', error);
        //console.log('ROSLIB Websocket error');
        if (ros !== undefined) {
            ros.close();
        }
        setTimeout(pollROS, shortDelay);
    });

    ros.on('close', function() {
        //console.log('Connection to websocket server closed.');
        webModelFunctions.scrollingStatusUpdate('ROSLIB Websocket closed');
        connectedToROS = false;
        //updateConnectedButton();
        setTimeout(pollROS, shortDelay);
    });
};

function start() {
    pollROS();
}

exports.start = start;
exports.setParam = setParam;
