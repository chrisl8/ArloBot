const webModel = require('../webModel');
const webModelFunctions = require('../webModelFunctions');
const robotModel = require('../robotModel');
const rosInterface = require('../rosInterface');
const textme = require('../textme');

let UnPlugRobotStarted = false;

async function unPlugRobot() {
    if (webModel.debugging) {
        console.log('Unplug Robot');
        webModelFunctions.scrollingStatusUpdate('Unplug Robot');
    }
    if (webModel.laptopFullyCharged) {
        if (webModel.unplugYourself) {
            if (!UnPlugRobotStarted) {
                webModelFunctions.update('status', 'Unplugging myself!');
                rosInterface.unplugRobot(true);
                webModelFunctions.scrollingStatusUpdate('Unplug Robot:  process starting!');
                UnPlugRobotStarted = true;
                return false;
            } else {
                // This should loop until the robot finishes unplugging itself.
                webModelFunctions.behaviorStatusUpdate('Unplug Robot:  unplugging . . .');
                return false;
            }
        } else {
            if (!robotModel.unplugMeTextSent) {
                textme('Please unplug me!');
                robotModel.unplugMeTextSent = true;
                webModelFunctions.scrollingStatusUpdate('Unplug Robot:  requesting assistance.');
            }
            return true;
        }
    } else {
        webModelFunctions.update('status', 'Charging . . .');
        webModelFunctions.behaviorStatusUpdate('Unplug Robot:  waiting for full charge.');
        // We cannot do much else until we are unplugged.
        return true;
    }
}

module.exports = unPlugRobot;
