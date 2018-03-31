const webModel = require('../webModel');
const webModelFunctions = require('../webModelFunctions');
const personalData = require('../personalData');
const robotModel = require('../robotModel');
const killROS = require('../killROS');

async function startROS() {
    if (webModel.debugging) {
        console.log('Start ROS');
        webModelFunctions.scrollingStatusUpdate('Start ROS');
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
                webModelFunctions.scrollingStatusUpdate('Start ROS: Process Closed.');
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
                return false;
            } else if (!webModel.ROSstart) {
                // IF we were told NOT to run, we need to stop the process,
                // and then wait for the failure to arrive here on the next loop.
                // Insert command to stop current function here:
                // This command must be OK with being called multiple times.
                killROS(false);
                // Don't change anything else,
                // Let the next loop fall into the "hasExited" option above.c
                return false;
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
                if (webModel.behaviorStatus === "Start ROS: Starting up . . .") {
                    webModelFunctions.behaviorStatusUpdate("ROS Startup Complete.");
                }
                // Now that ROS is running, other behaviors can continue.
                return true;
            }
        } else {
            webModelFunctions.behaviorStatusUpdate("Start ROS: Starting up . . .");
            return false;
        }
    } else if (webModel.ROSstart) {
        // IF the process is supposed to start, but wasn't,
        // then run it:
        webModelFunctions.update('status', 'Start ROS Requested.');
        if (!personalData.demoWebSite) {
            robotModel.ROSprocess.start();
        } else {
            robotModel.ROSprocess.started = true;
            setTimeout(() => {
                if (!robotModel.ROSprocess.startupComplete) {
                    robotModel.ROSprocess.startupComplete = true;
                    webModelFunctions.updateRosTopicItem('cliff', false);
                    webModelFunctions.updateRosTopicItem('floorObstacle', true);
                    webModelFunctions.updateRosTopicItem('safeToRecede', true);
                    webModelFunctions.updateRosTopicItem('safeToProceed', false);
                    webModelFunctions.updateRosTopicItem('Escaping', false);
                    webModelFunctions.updateRosTopicItem('minDistanceSensor', 2);
                    webModelFunctions.updateRosTopicItem('abd_speedLimit', 93);
                    webModelFunctions.updateRosTopicItem('abdR_speedLimit', 220);
                    webModelFunctions.updateRosTopicItem('acPower', true);
                    webModelFunctions.updateRosTopicItem('Heading', 0.000);
                    webModelFunctions.updateRosTopicItem('gyroHeading', 0.000);
                    webModelFunctions.updateRosTopicItem('leftMotorPower', true);
                    webModelFunctions.updateRosTopicItem('rightMotorPower', true);
                    webModelFunctions.updateRosTopicItem('laptopBatteryPercent', 85);
                    webModelFunctions.updateRosTopicItem('robotBatteryLevel', 12.6);
                    webModelFunctions.updateRosTopicItem('robotBatteryLow', false);
                }
            }, 2000);
        }
        webModelFunctions.scrollingStatusUpdate("Start ROS: Process starting!");
        return false;
    }
    // If the process isn't running and wasn't requested to run:
    webModelFunctions.behaviorStatusUpdate('Waiting for StartROS request.');
    return true;
}

module.exports = startROS;
