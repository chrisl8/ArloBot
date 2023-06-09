const webModel = require('../webModel');
const webModelFunctions = require('../webModelFunctions');
const robotModel = require('../robotModel');
const rosInterface = require('../rosInterface');
const pushMe = require('../pushMe');

let UnPlugRobotStarted = false;

async function unPlugRobot() {
  if (webModel.debugging && webModel.logBehaviorMessages) {
    const message = ' - Checking: Unplug Robot';
    console.log(message);
    webModelFunctions.scrollingStatusUpdate(message);
  }
  if (
    webModel.ROSisRunning &&
    webModel.pluggedIn &&
    (webModel.mapName !== '' || webModel.makeMapRunning)
  ) {
    if (webModel.laptopFullyCharged) {
      if (webModel.unplugYourself) {
        if (!UnPlugRobotStarted) {
          webModelFunctions.update('status', 'Unplugging myself!');
          rosInterface.unplugRobot(true);
          webModelFunctions.scrollingStatusUpdate(
            'Unplug Robot:  process starting!',
          );
          UnPlugRobotStarted = true;
          return false;
        }
        // This should loop until the robot finishes unplugging itself.
        webModelFunctions.behaviorStatusUpdate(
          'Unplug Robot:  unplugging . . .',
        );
        return false;
      }
      if (!robotModel.unplugMeTextSent) {
        pushMe('Please unplug me!');
        robotModel.unplugMeTextSent = true;
        webModelFunctions.scrollingStatusUpdate(
          'Unplug Robot:  requesting assistance.',
        );
      }
      return true;
    }
    webModelFunctions.update('status', 'Charging . . .');
    webModelFunctions.behaviorStatusUpdate(
      'Unplug Robot:  waiting for full charge.',
    );
  }
  // This behavior is idle, allow behave loop to continue to next entry.
  return true;
}

module.exports = unPlugRobot;
