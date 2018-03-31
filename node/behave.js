/*
 * This is the "behavior" of the rebot.
 * It should just be a very simple flow of function calls.
 *
 * This was once a fancy "Behavior" system using a library,
 * but the operation was complex and it required outside tools to view and modify.
 *
 * This aims to use simple program logic, so it should be clear if you read the code,
 * what the robot will do,
 * and you can just edit the code to change things.
 *
 * The only "rule" is to try to avoid putting any code here other than the logic and function calls.
 *
 */

const polling = require('./behaviors/polling');
const startROS = require('./behaviors/startROS');
const makeMap = require('./behaviors/makeMap');
const autoExplore = require('./behaviors/autoExplore');
const loadMap = require('./behaviors/loadMap');
const unPlugRobot = require('./behaviors/unPlugRobot');
const goToWaypoint = require('./behaviors/goToWaypoint');
const handlePowerWithoutROS = require('./behaviors/handlePowerWithoutROS');
// const gotoRandomLocation = require('behaviors/gotoRandomLocation');

const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
const wait = require('./wait');
const killROS = require('./killROS');

async function loop() {
    if (webModel.debugging) {
        console.log('Behave loop starting:');
    }
    // This is a bit of a catchall.
    // Certainly some things here could be broken out into their own behavior files.
    await polling();

    // The pattern is that if a behavior is "working on something" it returns false,
    // causing the loop to abort, so that it can keep working uninterrupted.
    // Otherwise it should always return true, to let the loop do the next thing.

    // A behavior should be callable over and over, even if it is already running.

    if (!await startROS()) {
        return;
    }

    if (webModel.ROSisRunning) {
        if (webModel.autoExplore) {
            if (!await autoExplore()) {
                return;
            }
        } else if (webModel.makeMap) {
            if (!await makeMap()) {
                return;
            }
        } else {
            // TODO: Only call if there is a map name, and make another behavior for "Where am I?"
            if (!await loadMap()) {
                return;
            }
        }

        if (webModel.pluggedIn && (webModel.autoExplore || webModel.mapName !== '')) {
            if (!await unPlugRobot()) {
                return;
            }
        }

        if (webModel.mapName !== '' && !webModel.pluggedIn) {
            if (webModel.wayPointNavigator.goToWaypoint) {
                if (!await goToWaypoint()) {
                    return;
                }
            } else {
                // Robot has a map loaded, and is not plugged in.
                // TODO: GO to random location
            }
        }
    } else {
        // ROS is NOT running
        await handlePowerWithoutROS();
    }
    // TODO: More stuff.

    if (webModel.debugging) {
        console.log("Nothing stopped this behave loop.");
    }

}

async function behave() {
    while (!webModel.shutdownRequested) {
        try {
            await loop();
        } catch (e) {
            console.error('Behavior Loop Error:');
            console.error(e);
        }
        await wait(1);
    }
    // This allows the script to kill itself.
    if (!webModel.killRosHasRun) {
        console.log('Shutdown Requested via webModel.');
    }
    webModelFunctions.behaviorStatusUpdate('Shutdown Requested via webModel.');
    killROS(true);
}

module.exports = behave;
