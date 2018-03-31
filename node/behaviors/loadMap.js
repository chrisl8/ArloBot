const webModel = require('../webModel');
const webModelFunctions = require('../webModelFunctions');
const robotModel = require('../robotModel');
const howManySecondsSince = require('../howManySecondsSince');
const textme = require('../textme');
const rosInterface = require('../rosInterface');
const WayPoints = require('../WayPoints.js');
const wayPointEditor = new WayPoints();
const LaunchScript = require('../LaunchScript');
const personalData = require('../personalData');

async function loadMap() {
    if (webModel.debugging) {
        console.log('Load Map');
        webModelFunctions.scrollingStatusUpdate('Load Map');
    }
    // ROS Process launch behavior pattern:
    // FIRST: This decides if we run this process or not:
    const delayTime = 60 * 5; // Five (5) minutes to find QR code.
    if (webModel.mapName === '') {
        if (!robotModel.whereamiTextSent && howManySecondsSince(robotModel.bootTime) >= delayTime) {
            textme('Where am I?');
            robotModel.whereamiTextSent = true;
        }
        // This fails if we have no map name,
        // and the tree loops again.
        return true;
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
                    webModelFunctions.scrollingStatusUpdate('Load Map' + "Process Closed.");
                    // 5. Set any special status flags for this
                    // process. i.e. ROSisRunning sets the start/stop button position
                    // NOTHING HERE
                    // 6. Any special "cleanup" required?
                    // This command must be OK with being called multiple times.
                    robotModel.initialPoseSet = false;
                    // Leave it 'RUNNING' and
                    // let the next Behavior tick respond as it would,
                    // if this function was never requested.
                    return false;
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
                                const set2dPoseEstimate = new LaunchScript({
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
                        return false;
                    }
                    if (robotModel.mapLoadTime === undefined) {
                        webModelFunctions.update('status', 'Map is Loaded.');
                        robotModel.mapLoadTime = new Date(); // Time that map was loaded.
                    }
                    // Whether we return 'RUNNING' or 'SUCCESS',
                    // is dependent on how this Behavior node works.
                    // The load map script stays running in the background,
                    // when it is "done", so we will call this SUCCESS.
                    return true;
                }
            } else {
                webModelFunctions.behaviorStatusUpdate('Load Map' + " Starting up . . .");
                return false;
            }
        } else {
            // IF the process is supposed to start, but wasn't,
            // then run it:
            webModelFunctions.scrollingStatusUpdate('Map: ' + webModel.mapName);
            //robotModel.loadMapProcess.scriptArguments = [webModel.mapName];
            robotModel.loadMapProcess.ROScommand = robotModel.loadMapProcess.ROScommand + process.env.HOME + '/.arlobot/rosmaps/' + webModel.mapName + '.yaml';
            webModelFunctions.scrollingStatusUpdate(robotModel.loadMapProcess.ROScommand);

            if (personalData.demoWebSite) {
              robotModel.loadMapProcess.started = true;
              robotModel.loadMapProcess.startupComplete = true;
            } else {
              robotModel.loadMapProcess.start();
            }
            return false;
        }
    }
}

module.exports = loadMap;
