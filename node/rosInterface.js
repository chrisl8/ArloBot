var webModel = require('./webModel');
var webModelFunctions = require('./webModelFunctions');
var robotModel = require('./robotModel');
var ROSLIB = require('roslib');

// Copied from arloweb.js
var connectedToROS = false, // Track my opinion of the connection
    ros, // Empty global for actual connection.
    shortDelay = 1000,
    longDelay = 3000

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
    ignoreFloorSensors: {
        param: null,
        label: 'ignoreFloorSensors',
        path: '/arlobot/ignoreFloorSensors'
    },
    monitorACconnection: {
        param: null,
        label: 'monitorACconnection',
        path: '/arlobot/monitorACconnection'
    },
    mapName: {
        param: null,
        label: 'mapName',
        path: '/arlobot/mapname'
    },
    explorePaused: {
        param: null,
        label: 'explorePaused',
        path: '/arlobot_explore/pause'
    }
};

var talkToROS = function() {
    // If you wanted to dump ALL params:
    //ros.getParams(function(params) {
    //    console.log('ROSLIB Params:');
    //    console.log(params);
    //});

    // Start subscriptions:
    // Each topic function will do its own checking to see if the topic is live or not.
    setTimeout(subscribeToActiveStatus, shortDelay); // Start with a slight delay

    // Enumerate parameters to watch
    for (var prop in rosParameters) {
        rosParameters[prop].param = new ROSLIB.Param({
            ros: ros,
            name: rosParameters[prop].path
        });
    }
    // and poll them.
    pollParams();
};

var pollParams = function() {
    function checkParameter(prop) {
        rosParameters[prop].param.get(function(value) {
            //console.log(rosParameters[prop].label + ': ' + value);
            // Assign state to webModel object for view by web page.
            if (webModel.rosParameters.hasOwnProperty(prop)) {
                webModelFunctions.updateRosParameter(prop, value);
                //webModel.rosParameters[prop] = value;
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

var closeDeadROSConnection = function() {
    // TODO: Does this ever happen?
    'use strict';
    console.log("Closing dead ROS connection.");
    if (ros !== undefined) {
        ros.close();
    }
    console.log("CLOSED dead ROS connection!");
};

var subscribeToActiveStatus = function() {
    'use strict';
    // Remember to add new instances to talkToROS() at the end!
    // This should serve as a template for all topic subscriptions
    // Make sure we are still connected.
    // NOT NEEDED IN NODE? THIS WAS FOR WEB SOCKET CONNECTIONS.
    // No need to recall myself as a new connect will do that.
    //if (!connectedToROS) {
    //    console.log('not connected');
    //    return;
    //}
    // Make sure service exists:
    var closeDeadConnectionTime;
    closeDeadConnectionTime = setTimeout(closeDeadROSConnection, longDelay);
    ros.getTopics(function(result) { // Unfortunately this can stall with no output!
        clearTimeout(closeDeadConnectionTime);
        // ONLY REQUIRED FOR WEB SOCKET CONNECTION?
        //if (!checkROSService(result.indexOf('/cmd_vel_mux/active'))) {
        //    console.log('topic dead');
        //    setTimeout(subscribeToMetatron_idStatus, longDelay);
        //    // Try again when all topics are up!
        //    return;
        //}

        // THIS is where you put the subscription code:
        var cmd_activeStatus = new ROSLIB.Topic({
            ros: ros,
            name: '/cmd_vel_mux/active', // Obtain name by running 'rostopic list'
            messageType: 'std_msgs/String' // Obtain Type by running 'rostopic info <name>'
        }); // Obtain message.??? by running 'rosmsg show <messageType>'

        cmd_activeStatus.subscribe(function(message) {
            robotModel.active_cmd = message.data;
            //console.log('Command Velocity Topic says: ' + message.data);
            if (message.data === 'idle') {
                robotModel.cmdTopicIdle = true;
            } else {
                robotModel.cmdTopicIdle = false;
            }
        });
    });
};

// Copied from arloweb.js
// Be sure to set url to point to localhost,
// and change any references to web objects with console.log (i.e. setActionField)
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
