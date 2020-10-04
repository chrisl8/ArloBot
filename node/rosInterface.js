const ROSLIB = require('roslib');
const personalData = require('./personalData');
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
const tts = require('./tts');
const robotModel = require('./robotModel');
// Set last movement to now to initiate the idle timer
robotModel.lastMovementTime = Date.now();

let unplug;

// Copied from arloweb.js
/** @namespace personalData.rosLibDelay */
let ros; // Empty global for actual connection.
const longDelay = personalData.rosLibDelay * 1000;

// Define a list of ROS Parameters to monitor
// NOTE: Add an instance to webModel if you want this sent to the web app!
const rosParameters = {
  ignoreCliffSensors: {
    param: webModel.rosParameters.ignoreCliffSensors,
    label: 'ignoreCliffSensors',
    path: '/arlobot/ignoreCliffSensors',
  },
  ignoreProximity: {
    param: webModel.rosParameters.ignoreProximity,
    label: 'ignoreProximity',
    path: '/arlobot/ignoreProximity',
  },
  ignoreIRSensors: {
    param: webModel.rosParameters.ignoreIRSensors,
    label: 'ignoreIRSensors',
    path: '/arlobot/ignoreIRSensors',
  },
  ignoreFloorSensors: {
    param: webModel.rosParameters.ignoreFloorSensors,
    label: 'ignoreFloorSensors',
    path: '/arlobot/ignoreFloorSensors',
  },
  monitorACconnection: {
    param: webModel.rosParameters.monitorACconnection,
    label: 'monitorACconnection',
    path: '/arlobot/monitorACconnection',
  },
  monitorDoors: {
    param: webModel.rosParameters.monitorDoors,
    label: 'monitorDoors',
    path: '/arlobot/monitorDoors',
  },
  mapName: {
    param: webModel.rosParameters.mapName,
    label: 'mapName',
    path: '/arlobot/mapname',
  },
};

function unplugRobot(value) {
  if (unplug) {
    // noinspection JSIgnoredPromiseFromCall
    tts('Unplugging myself now.');
    const unplugRequest = new ROSLIB.ServiceRequest({
      // args from rosservice info <service>
      unPlug: value, // Note javaScript uses true not True for bool
    });
    unplug.callService(unplugRequest, (result) => {
      console.log(result);
      // webModelFunctions.scrollingStatusUpdate(result);
    });
  }
}

function closeDeadROSConnection() {
  // TODO: Does this ever happen?
  console.log('Closing dead ROS connection.');
  if (ros !== undefined) {
    ros.close();
  }
  console.log('CLOSED dead ROS connection!');
}

function subscribeToActiveStatus() {
  // Remember to add new instances to talkToROS() at the end!
  // This should serve as a template for all topic subscriptions
  // Make sure we are still connected.

  // Make sure service exists:
  const closeDeadConnectionTime = setTimeout(closeDeadROSConnection, longDelay);
  ros.getTopics(() => {
    // Arguments: result // Unfortunately this can stall with no output!
    clearTimeout(closeDeadConnectionTime);

    // THIS is where you put the subscription code:
    const cmdActiveStatus = new ROSLIB.Topic({
      ros,
      name: '/cmd_vel_mux/active', // Obtain name by running 'rostopic list'
      messageType: 'std_msgs/String', // Obtain Type by running 'rostopic info <name>'
    }); // Obtain message.??? by running 'rosmsg show <messageType>'
    cmdActiveStatus.subscribe((message) => {
      robotModel.active_cmd = message.data;
      console.log(`Command Velocity Topic says: ${message.data}`);
      if (message.data === 'idle') {
        robotModel.cmdTopicIdle = true;
      } else {
        robotModel.cmdTopicIdle = false;
        robotModel.lastMovementTime = Date.now();
      }
    });

    // THIS is where you put the subscription code:
    const arlobotArloStatus = new ROSLIB.Topic({
      ros,
      name: '/arlo_status', // Obtain name by running 'rostopic list'
      messageType: 'arlobot_ros/arloStatus', // Obtain Type by running 'rostopic info <name>'
    }); // Obtain message.??? by running 'rosmsg show <messageType>'
    arlobotArloStatus.subscribe((message) => {
      for (const key in message) {
        if (message.hasOwnProperty(key)) {
          webModelFunctions.updateRosTopicItem(key, message[key]);
        }
      }
    });

    // THIS is where you put the subscription code:
    const arlobotSafetyStatus = new ROSLIB.Topic({
      ros,
      name: '/arlobot_safety/safetyStatus', // Obtain name by running 'rostopic list'
      messageType: 'arlobot_ros/arloSafety', // Obtain Type by running 'rostopic info <name>'
    }); // Obtain message.??? by running 'rosmsg show <messageType>'
    arlobotSafetyStatus.subscribe((message) => {
      for (const key in message) {
        if (message.hasOwnProperty(key)) {
          webModelFunctions.updateRosTopicItem(key, message[key]);
        }
      }
    });

    // THIS is where you put the subscription code:
    const arlobotJoystick = new ROSLIB.Topic({
      ros,
      name: '/joy', // Obtain name by running 'rostopic list'
      messageType: 'sensor_msgs/Joy', // Obtain Type by running 'rostopic info <name>'
    }); // Obtain message.??? by running 'rosmsg show <messageType>'
    arlobotJoystick.subscribe((message) => {
      // console.log(message.buttons);
      // A, B, X, Y, LB, RB, BACK, START, Xbox360, LeftStick, RightStick, Left, Right, Up, Down
      if (message.buttons[0] === 1) {
        // noinspection JSIgnoredPromiseFromCall
        tts('Hello, my name is two flower');
      } else if (message.buttons[1] === 1) {
        // noinspection JSIgnoredPromiseFromCall
        tts('What is your name?');
      } else if (message.buttons[3] === 1) {
        // noinspection JSIgnoredPromiseFromCall
        tts('Nice to meet you.');
      } else if (message.buttons[2] === 1) {
        // noinspection JSIgnoredPromiseFromCall
        tts('Excuse me.');
      } else if (message.buttons[5] === 1) {
        // noinspection JSIgnoredPromiseFromCall
        tts('~/.arlobot/sounds/readyMaster.wav');
      } else if (message.buttons[11] === 1) {
        // noinspection JSIgnoredPromiseFromCall
        tts('~/.arlobot/sounds/Exterminate.wav');
      } else if (message.buttons[12] === 1) {
        // noinspection JSIgnoredPromiseFromCall
        tts('~/.arlobot/sounds/input1.wav');
      } else if (message.buttons[13] === 1) {
        // noinspection JSIgnoredPromiseFromCall
        tts('~/.arlobot/sounds/affirmative.wav');
      } else if (message.buttons[14] === 1) {
        // noinspection JSIgnoredPromiseFromCall
        tts('~/.arlobot/sounds/depressed.wav');
      }
      /*
       for (let key in message) {
       if (message.hasOwnProperty(key)) {
       webModelFunctions.updateRosTopicItem(key, message[key]);
       }
       }
       */
    });
  });
}

function pollParams() {
  function checkParameter(prop) {
    rosParameters[prop].param.get((value) => {
      // console.log(rosParameters[prop].label + ': ' + value);
      // Assign state to webModel object for view by web page.
      if (webModel.rosParameters.hasOwnProperty(prop)) {
        webModelFunctions.updateRosParameter(prop, value);
        // webModel.rosParameters[prop] = value;
        // console.log('For web: ' + webModel.rosParameters[prop]);
      }
    });
  }

  for (const prop in rosParameters) {
    if (rosParameters.hasOwnProperty(prop)) {
      checkParameter(prop);
    }
  }

  setTimeout(pollParams, longDelay);
}

function talkToROS() {
  // If you wanted to dump ALL params:
  // ros.getParams(function(params) {
  //    console.log('ROSLIB Params:');
  //    console.log(params);
  // });

  // Start subscriptions:
  // Each topic function will do its own checking to see if the topic is live or not.
  setTimeout(subscribeToActiveStatus, longDelay); // Start with a slight delay

  // Set up services to use
  unplug = new ROSLIB.Service({
    ros,
    name: '/arlobot_unplug', // rosservice list
    serviceType: 'arlobot_ros/UnPlug', // rosservice info <service>
  });

  // Enumerate parameters to watch
  for (const prop in rosParameters) {
    if (rosParameters.hasOwnProperty(prop)) {
      rosParameters[prop].param = new ROSLIB.Param({
        ros,
        name: rosParameters[prop].path,
      });
    }
  }
  // and poll them.
  pollParams();
}

function setParam(paramLabel, value) {
  if (!personalData.demoWebSite) {
    if (rosParameters.hasOwnProperty(paramLabel)) {
      if (rosParameters[paramLabel].param) {
        rosParameters[paramLabel].param.set(value);
      }
    }
  }
  // Fake update for web interface to see until polling picks it up.
  // This also covers the case if this is a demo site.
  webModelFunctions.updateRosParameter(paramLabel, value);
}

// Copied from arloweb.js
// Be sure to set url to point to localhost,
// and change any references to web objects with console.log (i.e. setActionField)
function pollROS() {
  // console.log('ROSLIB pollROS run');

  ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090',
    // This eliminates a warning about utf8:
    encoding: 'ascii',
  });

  ros.on('connection', () => {
    webModelFunctions.scrollingStatusUpdate('ROSLIB Websocket connected.');
    // Set last movement to now to initiate the idle timer
    robotModel.lastMovementTime = Date.now();
    // Tell the system that ROS is up, even if it wasn't started via the script in Node
    webModelFunctions.update('ROSisRunning', true);
    // connectRequested = true;
    // updateConnectedButton();
    // checkROSServices();
    setTimeout(talkToROS, longDelay);
  });

  ros.on('error', () => {
    // Arguments: error
    // console.log('Error connecting to websocket server: ', error);
    // console.log('ROSLIB Websocket error');
    if (ros !== undefined) {
      ros.close();
    }
  });

  ros.on('close', () => {
    // console.log('Connection to ROSLIB Websocket server closed.');
    webModelFunctions.scrollingStatusUpdate('ROSLIB Websocket closed');
    // updateConnectedButton();
    setTimeout(pollROS, longDelay);
  });
}

function start() {
  pollROS();
}

exports.start = start;
exports.setParam = setParam;
exports.unplugRobot = unplugRobot;
