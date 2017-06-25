// 'use strict'; // I cannot do this until b3 is fixed. See the b3 = {}; line below.
const personalData = require('./personalData');
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
webModelFunctions.update('robotName', personalData.robotName);
const robotModel = require('./robotModel');
const speechEngine = require('./speechEngine');
const checkBattery = require('./checkBattery');
const masterRelay = require('./MasterRelay');
const UsbRelay = require('./UsbRelayControl');
robotModel.usbRelay = new UsbRelay();
const howManySecondsSince = require('./howManySecondsSince');
const Stochator = require('stochator');
const fs = require('fs');
/* Behavior3.js requires this global 'b3' variable.
 * It is an artifact of it being a web library,
 * not written for node.js.
 */
//b3 = {};
const b3 = require('./behavior3-0.2.0');
const exec = require('child_process').exec;
const spawn = require('child_process').spawn;
// Note that tts will convert text to speech,
// or it will send a ".wav" string (path, etc)
// to aplay.
// The benefit of using it is that it will honor
// system wide "bequiet" requests./usr/bin/upower -d|grep percentage|head -1
// The same module is used by ROS Python scripts.
const tts = require('./tts');
const LaunchScript = require('./LaunchScript');
const textme = require('./textme');
const rosInterface = require('./rosInterface');
const webserver = require('./webserver');
const os = require('os');
const repl = require('repl');
const handleSemaphoreFiles = require('./handleSemaphoreFiles');
const getQRcodes = require('./getQRcodes');
const saveScreenShotForWeb = require('./saveScreenShotForWeb');

const publishRobotURL = require('./publishRobotURL');

if (personalData.useBlueToothBeacon) {
    //noinspection JSUnusedLocalSymbols
    const bluetoothBeaconScanner = require('./BluetoothBeaconScanner');
}
const SocketServerSubscriber = require('./SocketServerSubscriber');
const RemoteMessageHandler = require('./RemoteMessageHandler');
const remoteMessageHandler = new RemoteMessageHandler();
const socketServerSubscriber = new SocketServerSubscriber(remoteMessageHandler.handleMessage);
socketServerSubscriber.start();

const WayPoints = require('./WayPoints.js');
const wayPointEditor = new WayPoints();

const arloTree = new b3.BehaviorTree();

rosInterface.start();

// Cleanup on shutdown
// http://stackoverflow.com/questions/14031763/doing-a-cleanup-action-just-before-node-js-exits
var kill_rosHasRun = false;

function killROS(exitWhenDone) {
    'use strict';
    var command = __dirname + '/../scripts/kill_ros.sh';
    // It is rather catastrophic if this repeats!
    if (!kill_rosHasRun) {
        kill_rosHasRun = true;
        webModelFunctions.update('ROSstart', false);
        // ROS drops all knowledge when restarted,
        // So make it clear to the user that he needs to pick a map again.
        webModelFunctions.update('mapName', '');
        webModelFunctions.update('autoExplore', false);
        webModelFunctions.scrollingStatusUpdate("Running kill_ros.sh . . .");
        // Logging to console too, because feedback on shutdown is nice.
        console.log("Running kill_ros.sh . . .");
        // and then also run the kill ROS command:
        var shutdownCommand = exec(command);
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
                kill_rosHasRun = false;
                webModelFunctions.update('ROSisRunning', false);
            }
        });
        shutdownCommand.on('error', function (err) {
            webModelFunctions.scrollingStatusUpdate('Shutdown process error' + err);
        });
    }
}

function exitHandler(options, err) {
    'use strict';
    if (options.cleanup) {
        console.log('Shutdown complete.');
        webModelFunctions.scrollingStatusUpdate('Shutdown complete.');
    }
    if (err) {
        console.log('Process Error:');
        console.log(err);
        console.log(err.stack);
        webModelFunctions.scrollingStatusUpdate(err.stack);
    }
    //if (options.exit) process.exit();
    if (options.exit) {
        webModelFunctions.scrollingStatusUpdate('Main process exit, calling killROS');
        console.log('Main process exit, calling killROS');
        killROS(true);
    }
}

//do something when app is closing
process.on('exit', exitHandler.bind(null, {
    cleanup: true
}));

//catches ctrl+c event
process.on('SIGINT', exitHandler.bind(null, {
    exit: true
}));

//catches uncaught exceptions
process.on('uncaughtException', exitHandler.bind(null, {
    exit: true
}));

webserver.start();

if (personalData.useMyCroft) {
    // The purpose of using myCroft to simply announce the robot's name is to validate that MyCroft is up.
    // Also, If you use the robot's name as a wake word and send it via TTS, MyCroft hears it and tries to respond.
    // Otherwise the normal speech engine is the better option for event based announcements.
    // MyCroft is primarily for interactive operations, such as asking the robot to do things.
    const myCroft = require('./MyCroft');
    webModelFunctions.update('myCroftIsRunning', true);
    myCroft.init();
    setTimeout(() => {
        // Give init time to finish.
        myCroft.injectText(`arlobotstartupskill ${personalData.robotName}`);
    }, 3000);
} else {
    tts(`Hello, my name is ${personalData.robotName}`);
}

// ## Here are the Behavior Tree Nodes ##
var intervalCount = 0; // Use this to throttle or space out polling items.
var intervalTop = 10;
var Poll = b3.Class(b3.Action);
Poll.prototype.name = 'Poll';
Poll.prototype.tick = function () { // Argument options: tick
    intervalCount++;
    if (intervalCount > intervalTop) {
        intervalCount = 0;
    }
    if (webModel.debugging) {
        console.log(this.name);
        webModelFunctions.scrollingStatusUpdate(this.name, intervalCount);
    }
    /* Some things just need to be polled, there is no way around it. Put those here.
     This only repeats about once per second, so it is a pretty good spacing, even without fancy code to slow things down
     but if you want to , set it to only happen when interval === some number
     below intervalTop.
     Or use some division to hit every odd/even number or multiple of 3.
     */
    handleSemaphoreFiles.readSemaphoreFiles();
    if (intervalCount === 5) {
        publishRobotURL.updateRobotURL();
    }
    if ((intervalCount % 2) === 0) {
        checkBattery();
    }
    if ((intervalCount % 2) === 1) {
        masterRelay('read');
        robotModel.usbRelay.updateAllRelayState();
    }
    if (intervalCount === intervalTop) {
        if (!webModel.cameraOn) {
            saveScreenShotForWeb();
        }
    }

    // Check for QR code:
    // If ROS has started, only do this when idle, but before ROS starts we can do it also,
    // that way it can have the map BEFORE ROS starts!
    // And also it won't text me with "Where am I?" if it is sitting in front of a QR code.
    // NOTE: At this point, once it gets an "unplug yourself" or "ROSstart" = true,
    // It will stop polling for QR codes.
    // But if we want to look for others later, remove "!webModel.hasSetupViaQRcode",
    // and it still will not set those two again (due to code in getQRcodes),
    // but it may fill in a map or fill in the webModel.QRcode line.
    // This may cause it to fight with other camera operations though.
    /** @namespace personalData.useQRcodes */
    if (intervalCount === 9 && !webModel.hasSetupViaQRcode && personalData.useQRcodes && !robotModel.gettingQRcode && !kill_rosHasRun && (robotModel.cmdTopicIdle || !webModel.ROSstart)) {
        // Old school thread control
        // It reduces how often zbarcam is run,
        // and prevents it from getting stuck
        robotModel.gettingQRcode = true;
        // TODO: This doesn't work with the Master & 5volt relays off!
        getQRcodes();
    }

    // If we are not finding a QRcode and no map is listed,
    // try turning on the light for a minute to see if it helps.
    // NOTE: Right now it won't do this if ROSstart is true,
    // assuming that if we started it manually, we don't want it to look
    // for a QR code for a map by itself
    var tryLightDelayTime = 60 * 2; // Two minutes
    if (!webModel.ROSstart && !webModel.hasSetupViaQRcode && !webModel.triedLightToFindQRcode && webModel.mapName === '' && howManySecondsSince(robotModel.bootTime) >= tryLightDelayTime && personalData.useQRcodes && !kill_rosHasRun) {
        webModelFunctions.update('triedLightToFindQRcode', true);
        spawn('../scripts/turn_on_light.sh');
        setTimeout(function () {
            if (!webModel.userLightOnRequested) {
                spawn('../scripts/turn_off_light.sh');
            }
        }, 60);
    }

    // Idle timer to shut off robot when left unattended
    /** @namespace personalData.idleTimeoutInMinutes */
    if (webModel.idleTimeout && personalData.idleTimeoutInMinutes > 0 && (webModel.ROSisRunning || webModel.masterRelayOn)) {
        // Set to now to fake out idle timer if no action is required.
        let lastActionDate = new Date();
        let dateNow = lastActionDate;
        // When ROS is active the idle timer is tied to the twist message command topic.
        if (webModel.ROSisRunning && robotModel.cmdTopicIdle) {
            lastActionDate = new Date(robotModel.lastMovementTime);
        }
        // When ROS is not active we are just wanting to shut off the main relay if it happens to be on by itself,
        // based on updates to the webModel
        if (!webModel.ROSisRunning && webModel.masterRelayOn) {
            lastActionDate = new Date(webModel.lastUpdateTime);
        }
        let idleMinutes = (dateNow - lastActionDate) / 1000 / 60;
        if (webModel.debugging) {
            console.log(`Idle Check: ${dateNow} - ${lastActionDate} = ${idleMinutes}`);
        }
        if (idleMinutes > personalData.idleTimeoutInMinutes) {
            if (webModel.ROSisRunning) {
                console.log("Idle shutdown.");
                webModelFunctions.scrollingStatusUpdate("Idle shutdown.");
                webModelFunctions.update('shutdownRequested', true);
            } else {
                console.log("Idle power down.");
                webModelFunctions.scrollingStatusUpdate("Idle power down.");
                masterRelay('off');
                robotModel.usbRelay.switchRelay('all', 'off');
            }
        }
    }
    /* TODO:
     2. Set up some sort of idle warning:
     a. Talk
     b. Web interface popup.
     */

    // This node will always return success,
    // although if you want to let some polling requirement
    // hang the robot until it is done you could return running
    // further up in that call.

    return b3.SUCCESS;
};

// TODO: Perfect this pattern and replicate to all script starting behaviors.
// TODO: Do we want it to eventually start ITSELF on load?
// maybe if it gets a certain signal, i.e. a QR code?
// TODO: Maybe it should text me asking me to let it start ROS?
var StartROS = b3.Class(b3.Action);
StartROS.prototype.name = 'StartROS';
StartROS.prototype.tick = function () { // Argument options: tick
    if (webModel.debugging) {
        console.log(this.name);
        webModelFunctions.scrollingStatusUpdate(this.name);
    }
    // ROS Process launch behavior pattern:
    // FIRST: Is the process already started?
    if (robotModel.ROSprocess.started) {
        // startupComplete indicates either:
        // Script exited
        // Script threw "success string"
        // Script returned any data if it wasn't given a "success string"
        if (robotModel.ROSprocess.startupComplete) {
            if (robotModel.ROSprocess.hasExited) {
                // Once the process has exited:
                // 1. DISABLE whatever user action causes it to be called,
                // so that it won't loop.
                webModelFunctions.update('ROSstart', false);
                // 2. Now that it won't loop, set .started to false,
                // so that it can be run again.
                robotModel.ROSprocess.started = false;
                // 3. Send a status to the web site:
                webModelFunctions.update('status', 'ROS process has closed.');
                // 4. Log the closure to the console,
                // because this is significant.
                webModelFunctions.scrollingStatusUpdate(this.name + "Process Closed.");
                // 5. Set any special status flags for this
                // process. i.e. ROSisRunning sets the start/stop button position
                webModelFunctions.update('ROSisRunning', false);
                // 6. Any special "cleanup" required?
                // In this case we will run the kill routine.
                // This command must be OK with being called multiple times.
                killROS(false);
                // Leave it 'RUNNING' and
                // let the next Behavior tick respond as it would,
                // if this function was never requested.
                return b3.RUNNING;
            } else if (!webModel.ROSstart) {
                // IF we were told NOT to run, we need to stop the process,
                // and then wait for the failure to arrive here on the next loop.
                // Insert command to stop current function here:
                // This command must be OK with being called multiple times.
                killROS(false);
                // Don't change anything else,
                // Let the next loop fall into the "hasExited" option above.c
                return b3.RUNNING;
            } else {
                // This is where we go if the start is complete,
                // and did not fail.
                // and we still want it running.
                // This will repeat on every tick!
                // 1. Set any special status flags for this
                // process. i.e. ROSisRunning sets the start/stop button position
                // Update last idle time to prevent instant idle timeout!
                if (!webModel.ROSisRunning) {
                    robotModel.lastMovementTime = Date.now();
                    webModelFunctions.update('status', 'ROS is Running.');
                }
                webModelFunctions.update('ROSisRunning', true);
                if (robotModel.startROSTime === undefined) {
                    robotModel.startROSTime = new Date(); // Time that ROS start was completed.
                }
                // Whether we return 'RUNNING' or 'SUCCESS',
                // is dependent on how this Behavior node works.
                // StartROS stays running in the background when it is "done",
                // so this is SUCCESS.
                return b3.SUCCESS;
            }
        } else {
            webModelFunctions.behaviorStatusUpdate(this.name + " Starting up . . .");
            return b3.RUNNING;
        }
    } else if (webModel.ROSstart) {
        // IF the process is supposed to start, but wasn't,
        // then run it:
        webModelFunctions.update('status', 'ROS Start Requested.');
        robotModel.ROSprocess.start();
        webModelFunctions.scrollingStatusUpdate(this.name + " Process starting!");
        return b3.RUNNING;
    }
    // If the process isn't running and wasn't requested to run:
    webModelFunctions.behaviorStatusUpdate('Waiting for StartROS request.');
    return b3.FAILURE;
};

var AutoExplore = b3.Class(b3.Action);
AutoExplore.prototype.name = 'AutoExplore';
AutoExplore.prototype.tick = function () { // Argument options: tick
    if (webModel.debugging) {
        console.log(this.name);
        webModelFunctions.scrollingStatusUpdate(this.name);
    }
    if (webModel.autoExplore) {
        if (robotModel.exploreProcess.started) {

            // Catch changes in pauseExplore and send them to the arlobot_explore pause_explorer service
            if (webModel.rosParameters.explorePaused !== null && webModel.rosParameters.explorePaused !== webModel.pauseExplore) {
                if (webModel.pauseExplore) {
                    webModelFunctions.scrollingStatusUpdate('Pausing Explorer...');
                } else {
                    webModelFunctions.scrollingStatusUpdate('Resuming Explorer...');
                }
                rosInterface.callPauseExplore(webModel.pauseExplore);
            }

            if (robotModel.exploreProcess.startupComplete) {
                if (robotModel.exploreProcess.hasExited) {
                    webModelFunctions.update('status', 'Explore process is closed.');
                    webModelFunctions.update('autoExplore', false);
                    robotModel.exploreProcess.started = false;
                    webModelFunctions.behaviorStatusUpdate(this.name + "FAILURE");
                    return b3.FAILURE;
                } else {
                    webModelFunctions.update('status', 'Explore process started.');
                    if (webModel.pluggedIn) {
                        webModelFunctions.behaviorStatusUpdate(this.name + 'Robot is still plugged in!');
                        // Returning success will allow the UnPlug Behavior to run
                        return b3.SUCCESS;
                    } else {
                        webModelFunctions.behaviorStatusUpdate(this.name + 'Robot is Exploring!');
                        return b3.RUNNING;
                    }
                }
            } else {
                webModelFunctions.update('status', 'Explore process is starting...');
                return b3.RUNNING;
            }
        } else {
            robotModel.exploreProcess.start();
            webModelFunctions.behaviorStatusUpdate(this.name);
            return b3.RUNNING;
        }
    } else {
        // Return FAILURE if we were NOT asked to explore,
        // thus passing priority on to LoadMap
        return b3.FAILURE;
    }
};

var LoadMap = b3.Class(b3.Action);
LoadMap.prototype.name = 'LoadMap';
LoadMap.prototype.tick = function () { // Argument options: tick
    if (webModel.debugging) {
        console.log(this.name);
        webModelFunctions.scrollingStatusUpdate(this.name);
    }
    // ROS Process launch behavior pattern:
    // FIRST: This decides if we run this process or not:
    var delayTime = 60 * 5; // Five (5) minutes to find QR code.
    if (webModel.mapName === '') {
        if (!robotModel.whereamiTextSent && howManySecondsSince(robotModel.bootTime) >= delayTime) {
            textme('Where am I?');
            robotModel.whereamiTextSent = true;
        }
        // This fails if we have no map name,
        // and the tree loops again.
        return b3.FAILURE;
    } else {
        if (robotModel.loadMapProcess.started) {
            if (robotModel.loadMapProcess.startupComplete) {
                if (robotModel.loadMapProcess.hasExited) {
                    // Once the process has exited:
                    // 1. DISABLE whatever user action causes it to be called,
                    // so that it won't loop.
                    webModelFunctions.update('mapName', '');
                    // 2. Now that it won't loop, set .started to false,
                    // so that it can be run again.
                    robotModel.loadMapProcess.started = false;
                    // 3. Send a status to the web site:
                    webModelFunctions.update('status', 'Map process has closed.');
                    // 4. Log the closure to the console,
                    // because this is significant.
                    webModelFunctions.scrollingStatusUpdate(this.name + "Process Closed.");
                    // 5. Set any special status flags for this
                    // process. i.e. ROSisRunning sets the start/stop button position
                    // NOTHING HERE
                    // 6. Any special "cleanup" required?
                    // This command must be OK with being called multiple times.
                    robotModel.initialPoseSet = false;
                    // Leave it 'RUNNING' and
                    // let the next Behavior tick respond as it would,
                    // if this function was never requested.
                    return b3.RUNNING;
                } else {
                    // This will repeat on every tick!
                    if (webModel.rosParameters.mapName !== webModel.mapName) {
                        rosInterface.setParam('mapName', webModel.mapName);
                    }
                    if (!robotModel.initialPoseSet) {
                        // webserver.js will populate webModel.wayPoints,
                        // when the map is set.
                        // If there is a waypoint called 'initial'
                        // use it to set the initial 2D pose estimate.
                        // http://answers.ros.org/question/9686/how-to-programatically-set-the-2d-pose-on-a-map/?answer=14155#post-id-14155
                        // http://wiki.ros.org/amcl#Subscribed_Topics
                        // Run RVIZ,
                        // rostopic echo initialpose
                        // Set inital pose and look at the output.
                        // You might need to use an online YAML to JSON converter
                        /// to get the right format for the command line.
                        if (webModel.wayPoints.indexOf('initial') > -1) {
                            wayPointEditor.getWayPoint('initial', function (response) {
                                var set2dPoseEstimate = new LaunchScript({
                                    debugging: true,
                                    name: 'set2dPoseEstimate',
                                    ROScommand: 'unbuffer rostopic pub -1 initialpose geometry_msgs/PoseWithCovarianceStamped "{ header: { seq: 0, stamp: { secs: 0, nsecs: 0 }, frame_id: map }, pose: { ' + response + ', covariance: [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ] } }"'
                                });
                                set2dPoseEstimate.start();
                            });
                        }
                        // Either the 2D pose estimate is set now,
                        // or if we do not have an "initial" pose, assume we started at map point 0
                        // TODO: This has an exit code, so can we set it to true when that happens?
                        // and keep returning "RUNNING" until it is set to true?!
                        /* This is the output:
                         set2dPoseEstimate is starting up . . .
                         set2dPoseEstimate stdout data:publishing and latching message for 3.0 seconds
                         set2dPoseEstimate exited with code: 0
                         */

                        robotModel.initialPoseSet = true;
                        // Give it one "loop" to get this done
                        return b3.RUNNING;
                    }
                    if (robotModel.mapLoadTime === undefined) {
                        webModelFunctions.update('status', 'Map is Loaded.');
                        robotModel.mapLoadTime = new Date(); // Time that map was loaded.
                    }
                    // Whether we return 'RUNNING' or 'SUCCESS',
                    // is dependent on how this Behavior node works.
                    // The load map script stays running in the background,
                    // when it is "done", so we will call this SUCCESS.
                    return b3.SUCCESS;
                }
            } else {
                webModelFunctions.behaviorStatusUpdate(this.name + " Starting up . . .");
                return b3.RUNNING;
            }
        } else {
            // IF the process is supposed to start, but wasn't,
            // then run it:
            webModelFunctions.scrollingStatusUpdate('Map: ' + webModel.mapName);
            //robotModel.loadMapProcess.scriptArguments = [webModel.mapName];
            robotModel.loadMapProcess.ROScommand = robotModel.loadMapProcess.ROScommand + process.env.HOME + '/.arlobot/rosmaps/' + webModel.mapName + '.yaml';
            webModelFunctions.scrollingStatusUpdate(robotModel.loadMapProcess.ROScommand);
            robotModel.loadMapProcess.start();
            return b3.RUNNING;
        }
    }
};

var UnPlugRobotStarted = false;
var UnPlugRobot = b3.Class(b3.Action);
UnPlugRobot.prototype.name = 'UnPlugRobot';
UnPlugRobot.prototype.tick = function () { // Argument options: tick
    if (webModel.debugging) {
        console.log(this.name);
        webModelFunctions.scrollingStatusUpdate(this.name);
    }
    if (!webModel.pluggedIn) return b3.SUCCESS;
    if (webModel.laptopFullyCharged) {
        if (webModel.unplugYourself) {
            if (!UnPlugRobotStarted) {
                webModelFunctions.update('status', 'Unplugging myself!');
                rosInterface.unplugRobot(true);
                webModelFunctions.scrollingStatusUpdate(this.name + " process starting!");
                UnPlugRobotStarted = true;
                return b3.RUNNING;
            } else {
                // This should loop until the robot finishes unplugging itself.
                webModelFunctions.behaviorStatusUpdate(this.name + " unplugging . . .");
                return b3.RUNNING;
            }
        } else {
            if (!robotModel.unplugMeTextSent) {
                textme('Please unplug me!');
                robotModel.unplugMeTextSent = true;
                webModelFunctions.scrollingStatusUpdate(this.name + " requesting assistance.");
            }
            return b3.FAILURE;
        }
    } else {
        webModelFunctions.update('status', 'Charging . . .');
        webModelFunctions.behaviorStatusUpdate(this.name + " waiting for full charge.");
        // We cannot do much else until we are unplugged.
        return b3.FAILURE;
    }
};

/*
 ##### Jobs MemPriority #####
 */

// TODO: Perfect this pattern and replicate to all script starting behaviors.
// TODO: Maybe it should text me asking me to let it start ROS?
var GoToWaypoint = b3.Class(b3.Action);
GoToWaypoint.prototype.name = 'GoToWaypoint';
GoToWaypoint.prototype.tick = function () { // Argument options: tick
    if (webModel.debugging) {
        console.log(this.name);
        webModelFunctions.scrollingStatusUpdate(this.name);
    }
    // ROS Process launch behavior pattern:
    // FIRST: Is the process already started?
    if (robotModel.goToWaypointProcess.started) {
        console.log('robotModel.goToWaypointProcess.started');
        // startupComplete indicates either:
        // Script exited
        // Script threw "success string"
        // Script returned any data if it wasn't given a "success string"
        // If your process just runs forever if it is "GOOD" then it will not
        // exit until we are DONE, and it should "SU"
        // but if it is a run, wait for finish, succeed type script,
        // we should RUNNING until .hasExited
        // NOTE:
        // Seriously understand this so you can know where to return
        // RUNNING <-
        // SUCCESS <- Not usually even used by "RUN, WAIT, RETURN" behaviors,
        //          but returned every loop by PERPETUAL behavior scripts.
        // FAILURE <- Usually means we aren't do things.
        if (robotModel.goToWaypointProcess.startupComplete) {
            console.log('robotModel.goToWaypointProcess.startupComplete');
            if (robotModel.goToWaypointProcess.hasExited) {
                console.log('robotModel.goToWaypointProcess.hasExited');
                // Once the process has exited:
                // 1. DISABLE whatever user action causes it to be called,
                // so that it won't loop.
                webModelFunctions.updateWayPointNavigator('mostRecentArrival', webModel.wayPointNavigator.wayPointName);
                webModelFunctions.updateWayPointNavigator('goToWaypoint', false);
                // 2. Now that it won't loop, set .started to false,
                // so that it can be run again.
                robotModel.goToWaypointProcess.started = false;
                // 3. Send a status to the web site:
                webModelFunctions.update('status', 'Arrived at ' + webModel.wayPointNavigator.wayPointName);
                // 4. Log the closure to the console,
                // because this is significant.
                webModelFunctions.scrollingStatusUpdate(this.name + "Process Closed.");
                // 5. Set any special status flags for this
                // process. i.e. ROSisRunning sets the start/stop button position
                //  NONE
                // 6. Any special "cleanup" required?
                //  NONE
                // Leave it 'RUNNING' and
                // let the next Behavior tick respond as it would,
                // if this function was never requested.
                return b3.RUNNING;
            } else if (!webModel.wayPointNavigator.goToWaypoint) {
                // KILL a node here if you want it to STOP!
                // Otherwise this is a non-event,
                // Either way the response should probably be RUNNING.
                console.log('!webModel.wayPointNavigator.goToWaypoint');
                // IF we were told NOT to run, we need to stop the process,
                // and then wait for the failure to arrive here on the next loop.
                // Insert command to stop current function here:
                // This command must be OK with being called multiple times.
                //  TODO: If we need to kill the goToWayPoint, do it here.
                // Don't change anything else,
                // Let the next loop fall into the "hasExited" option above.c
                if (webModel.debugging) {
                    console.log(this.name + ' RUNNING');
                    webModelFunctions.scrollingStatusUpdate(this.name + ' RUNNING');
                }
                return b3.RUNNING;
            } else {
                // LOOK HERE!
                // If this is a "PERPETUAL" process, then this is where you want
                // to return "SUCCESS" because "starupComplete" is true,
                // but it has not exited!
                // If this is a "RUN, WAIT, RETURN" process, then this is where
                // you want to return RUNNING to let behavior tree know
                // that we are IN PROCESS!
                console.log('GoToWaypoint startup complete without failure.');
                // This is where we go if the start is complete,
                // and did not fail.
                // and we still want it running.
                // This will repeat on every tick!
                // 1. Set any special status flags for this
                // process. i.e. ROSisRunning sets the start/stop button position
                //  NONE for GoToWaypoint
                // Whether we return 'RUNNING' or 'SUCCESS',
                // is dependent on how this Behavior node works.
                // GoToWaypoint should exit when the task is done,
                // so we call this running:
                if (webModel.debugging) {
                    console.log(this.name + ' RUNNING');
                    webModelFunctions.scrollingStatusUpdate(this.name + ' RUNNING');
                }
                return b3.RUNNING;
            }
        } else {
            webModelFunctions.behaviorStatusUpdate(this.name + " Starting up . . .");
            console.log(this.name + ' RUNNING');
            return b3.RUNNING;
        }
    } else if (webModel.wayPointNavigator.goToWaypoint) {
        console.log('webModel.wayPointNavigator.goToWaypoint');
        // IF the process is supposed to start, but wasn't,
        // then run it:
        webModelFunctions.update('status', 'Going to waypoint ' + webModel.wayPointNavigator.wayPointName);
        robotModel.goToWaypointProcess.ROScommand = 'unbuffer rosservice call /arlobot_goto/go_to_goal "' + robotModel.wayPointNavigator.destinaitonWaypoint + '"';
        robotModel.goToWaypointProcess.start();
        webModelFunctions.scrollingStatusUpdate(this.name + " Process starting!");
        return b3.RUNNING;
    }
    // If the process isn't running and wasn't requested to run:
    // Then this node has no action and we can carry on.
    return b3.FAILURE;
};

/* RESULTS from GoToWaypoint logging:
 webModel.wayPointNavigator.goToWaypoint
 Running GoToWaypoint child process . . .
 GoToWaypoint is starting up . . .
 robotModel.goToWaypointProcess.started
 GoToWaypoint RUNNING
 robotModel.goToWaypointProcess.started
 GoToWaypoint RUNNING
 robotModel.goToWaypointProcess.started
 GoToWaypoint RUNNING
 robotModel.goToWaypointProcess.started
 GoToWaypoint RUNNING
 robotModel.goToWaypointProcess.started
 GoToWaypoint RUNNING
 robotModel.goToWaypointProcess.started
 GoToWaypoint RUNNING
 robotModel.goToWaypointProcess.started
 GoToWaypoint RUNNING
 robotModel.goToWaypointProcess.started
 GoToWaypoint RUNNING
 robotModel.goToWaypointProcess.started
 GoToWaypoint RUNNING
 robotModel.goToWaypointProcess.started
 GoToWaypoint RUNNING
 GoToWaypoint stdout data:result: True
 GoToWaypoint exited with code: 0
 robotModel.goToWaypointProcess.started
 robotModel.goToWaypointProcess.startupComplete
 robotModel.goToWaypointProcess.hasExited
 */

/*
 ##### Idle MemPriority #####
 */

// This will only fail when we feel the robot is idle,
// allowing the rest of the items in this tree to take a shot,
// this way EVERY entry does not have to check for "idle"
// What about things that should happen sooner than later?
// Well we can also create an "idleTime" variable, and then
// nodes can key off of that too for further control.
var IsNotIdle = b3.Class(b3.Action);
IsNotIdle.prototype.name = 'IsNotIdle';
IsNotIdle.prototype.tick = function () { // Argument options: tick
    if (webModel.debugging) {
        console.log(this.name);
        webModelFunctions.scrollingStatusUpdate(this.name);
    }
    // For now I'm just going to "stall" for 3 minutes by setting a time.
    var repeatDelay = 60 * 3; // Three minutes.
    // Set initial "last run" time to now, so it waits the repeatDealy at least once?
    if (blackboard.get('lastRanTime', arloTree.id, arloTree.id) === undefined) {
        blackboard.set('lastRanTime', new Date(), arloTree.id, arloTree.id);
    }
    // We have to have more than one destination. :) and the initial pose set.
    if (robotModel.initialPoseSet && howManySecondsSince(blackboard.get('lastRanTime', arloTree.id, arloTree.id)) >= repeatDelay) {
        // If we are idle "FAIL" so that the rest of the items can try.
        // TODO: Uncomment this and keep working: console.log('Idle');
        return b3.FAILURE;
    } else {
        // TODO: Uncomment this and keep working: console.log('NOT idle!');
        return b3.RUNNING;
    }

};

var GotoRandomLocation = b3.Class(b3.Action);
GotoRandomLocation.prototype.name = 'GotoRandomLocation';
GotoRandomLocation.prototype.tick = function () { // Argument options: tick
    if (webModel.debugging) {
        console.log(this.name);
        webModelFunctions.scrollingStatusUpdate(this.name);
    }
    var repeatDelay = 60 * 10; // Ten minutes.
    // We have to have more than one destination. :) and the initial pose set.
    if (webModel.wayPoints.length > 1 && robotModel.initialPoseSet && howManySecondsSince(blackboard.get('lastRanTime', arloTree.id, arloTree.id)) >= repeatDelay) {
        console.log('GotoRandomLocation started:');
        console.log('Time since last call: ' + howManySecondsSince(blackboard.get('lastRanTime', arloTree.id, arloTree.id)));
        console.log('Previous destination: ' + blackboard.get('lastDestination', arloTree.id, arloTree.id));
        var destinationPicker = new Stochator({
            kind: "set",
            values: webModel.wayPoints
        });
        var destination;
        do {
            destination = destinationPicker.next();
        } while (destination === blackboard.get('lastDestination', arloTree.id, arloTree.id));
        // ^^^ To prevent going to the same place agian.
        blackboard.set('lastDestination', destination, arloTree.id, arloTree.id);
        blackboard.set('lastRanTime', new Date(), arloTree.id, arloTree.id);
        console.log('New Destination: ' + blackboard.get('lastDestination', arloTree.id, arloTree.id));
        console.log('--------------------------------');
        return b3.RUNNING;
    }
    return b3.FAILURE; // Place holder for now.
    //if (!webModel.pluggedIn) return b3.SUCCESS;
    //if (webModel.laptopFullyCharged) {
    //    if (webModel.unplugYourself) {
    //        if (!robotModel.unplugProcess.started) {
    //            webModel.status = 'Unplugging myself!';
    //            robotModel.unplugProcess.start();
    //            webModelFunctions.scrollingStatusUpdate(this.name + " process starting!");
    //            return b3.RUNNING;
    //        } else {
    //            // This should loop until the robot finishes unplugging itself.
    //            webModelFunctions.behaviorStatusUpdate(this.name + " unplugging . . .");
    //            return b3.RUNNING;
    //        }
    //    } else {
    //        if (!robotModel.unplugMeTextSent) {
    //            textme('Please unplug me!');
    //            robotModel.unplugMeTextSent = true;
    //            webModelFunctions.scrollingStatusUpdate(this.name + " requesting assistance.");
    //        }
    //        return b3.FAILURE;
    //    }
    //} else {
    //    webModel.status = 'Charging . . .';
    //    webModelFunctions.behaviorStatusUpdate(this.name + " waiting for full charge.");
    //    // We cannot do much else until we are unplugged.
    //    return b3.FAILURE;
    //}
};

// Build this file with http://behavior3js.guineashots.com/editor/#
// and you can LOAD this data into the editor to start where you left off again
const arloNodeData = require('./arloTreeData');
// var arloNodeData = JSON.parse(fs.readFileSync('arloTreeData.json', 'utf8'));

// Despite the Editor creating a beautiful JSON behavior tree for us,
// we still have to list the custom nodes for it by hand like this:
//var customNodeNames = {
//'ROSisRunning': ROSisRunning,
//'StartROS': StartROS,
//'UnPlugRobot': UnPlugRobot,
//'LaptopBatteryCharged': LaptopBatteryCharged,
//'RobotIsUnplugged': RobotIsUnplugged
//};
// but I'm using eval to automate the build here:
var customNodeNames = {};

function parseCustomNodes(element) { // Argument options: element, index, array
    'use strict';
    customNodeNames[element.name] = eval(element.name); // jshint ignore:line
}
arloNodeData.custom_nodes.forEach(parseCustomNodes);
if (webModel.debugging) {
    console.log(customNodeNames);
    webModelFunctions.scrollingStatusUpdate(customNodeNames);
}
arloTree.load(arloNodeData, customNodeNames);


// ## Scripts and ROS Commands that will be called by nodes ##
// NOTE: Be sure to put 'unbuffer ' at the beginning of any ROScommand
// if you hope to monitor the output, otherwise they never flush!
// http://stackoverflow.com/a/11337310
// http://linux.die.net/man/1/unbuffer

robotModel.ROSprocess = new LaunchScript({
    name: 'ROS',
    scriptName: '../scripts/start-robot.sh',
    successString: ']: Propellerbot_node has started.'
    // TODO: Could this be set to "true" by the ROSlibJS connection instead??
});

/* GotoWaypoint Process output:
 Running GoToWaypoint child process . . .
 GoToWaypoint is starting up . . .
 GoToWaypoint stdout data:result: True
 GoToWaypoint exited with code: 0

 So just wait for a clean exit, no need for a successString.
 */
robotModel.goToWaypointProcess = new LaunchScript({
    name: 'GoToWaypoint'
    // Set the ROScommmand at call time!
});

var exploreCommand;
if (personalData.use_xv11) {
    exploreCommand = 'unbuffer roslaunch arlobot_launchers add_autonomous_explore_xv11.launch';
} else {
    exploreCommand = 'unbuffer roslaunch arlobot_launchers add_autonomous_explore.launch';
}
robotModel.exploreProcess = new LaunchScript({
    name: 'Explore',
    ROScommand: exploreCommand,
    successString: 'odom received'
});

var loadMapCommand;
if (personalData.use_xv11) {
    loadMapCommand = 'unbuffer roslaunch arlobot_launchers load_map_xv11.launch map_file:=';
} else {
    loadMapCommand = 'unbuffer roslaunch arlobot_launchers load_map.launch map_file:=';
}
robotModel.loadMapProcess = new LaunchScript({
    name: 'LoadMap',
    ROScommand: loadMapCommand,
    successString: 'odom received'
});

var blackboard = new b3.Blackboard();

webModelFunctions.update('status', 'Behavior Tree is running.');

// This is where the behavior tree actually runs ("loops")
setInterval(function () {
    arloTree.tick(robotModel, blackboard);
    // Note that we can do stuff between ticks if we want to,
    // although tracking information in the arloBot object,
    // or node data in the blackboard is preferable.
    //console.log(blackboard);

    // This allows the script to kill itself.
    if (webModel.shutdownRequested) {
        if (!kill_rosHasRun) {
            console.log('Shutdown Requested via webModel.');
        }
        webModelFunctions.behaviorStatusUpdate('Shutdown Requested via webModel.');
        killROS(true);
    }

}, 1000);

console.log('Go to: http://' + os.hostname() + ':' + personalData.webServerPort + '/');

// Run Firefox with this page on the local machine,
// to provide a "menu" on the robot!
if (personalData.launchBrowser) {
    var runFirefox = new LaunchScript({
        name: 'FireFox',
        scriptName: '../scripts/runFirefox.sh',
        scriptArguments: 'http://' + os.hostname() + ':' + personalData.webServerPort + '/'
    });
    runFirefox.start();
}

// REPL server at the console for command line interaction and debugging:
var net = require("net");
var REPLconnections = 0;

// TODO: This outputs to the console, even if the REPL is via a socket. :)
function replHelp() {
    'use strict';
    console.log('Usage:\nwebModel - List webModel object\nrobotModel - List robotModel object\npersonalData - List personalData contents\n.exit - Shut down robot and exit.\nYou can also set object variables.');
    return '';
}
// TODO: There is no reason we cannot have a local AND remote REPL,
// but the remote makes using PM2 a possibility.

// https://nodejs.org/api/repl.html
// telnet localhost 5001
net.createServer(function (socket) {
    REPLconnections += 1;
    var replNetwork = repl.start({
        prompt: webModel.robotName + " > ",
        input: socket,
        output: socket
    }).on('exit', function () {
        socket.end();
    }).on('error', function (err) {
        console.log(err);
        socket.end();
    });
    // TODO: Do we have to have these in here?
    // can we declare a REPL external to the net.createServer,
    // and use it in the local and remote REPL?
    // Or is this how it must be?
    replNetwork.context.webModel = webModel;
    replNetwork.context.robotModel = robotModel;
    replNetwork.context.personalData = personalData;
    replNetwork.context.killROS = killROS;
    replNetwork.context.help = replHelp;
}).listen(5001);

var replConsole = repl.start({
    prompt: webModel.robotName + " > "
}).on('exit', function () {
    console.log('Got "exit" event from repl!');
    killROS(true);
}).on('error', function (err) {
    console.log(err);
    socket.end();
});
// TODO: Do we have to have these in here?
// can we declare a REPL external to the net.createServer,
// and use it in the local and remote REPL?
// Or is this how it must be?
replConsole.context.webModel = webModel;
replConsole.context.robotModel = robotModel;
replConsole.context.personalData = personalData;
replConsole.context.killROS = killROS;
replConsole.context.help = replHelp;
console.log('Press Enter to get a prompt.');
console.log('Run \'help()\' for a list of options.');
console.log('Use Ctrl+d to shut down robot.');
