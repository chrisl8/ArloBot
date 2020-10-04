const webModel = require('../webModel');
const webModelFunctions = require('../webModelFunctions');
const robotModel = require('../robotModel');

async function makeMap() {
  if (webModel.debugging && webModel.logBehaviorMessages) {
    const message = ' - Checking: Make Map';
    console.log(message);
    webModelFunctions.scrollingStatusUpdate(message);
  }
  if (robotModel.makeMap.started) {
    if (robotModel.makeMap.startupComplete) {
      if (robotModel.makeMap.hasExited) {
        webModelFunctions.update('status', 'Make Map process is closed.');
        webModelFunctions.update('makeMapRunning', false); // NOTE: Shared with all Make Map behaviors
        webModelFunctions.update('makeMap', false);
        robotModel.makeMap.started = false;
        webModelFunctions.behaviorStatusUpdate('Make Map: FAILURE');
        return false;
      }
      webModelFunctions.update('status', 'Make Map process started.');
      webModelFunctions.update('makeMapRunning', true); // NOTE: Shared with all Make Map behaviors
      // TODO: The initial robot pose should be saved to the pose list.
      //       In order to easily return the robot here after making the map,
      //       both with hand and automatic map creations.
      //       In theory it is all 0's? Except you cannot send
      //       the robot to an all 0 position. :shrug:
      //       It probably makes sense to ALWAYS set this as the "Dock" waypoint too.
      // TODO: It may be that simply setting w: 1.000 is all that is needed though, and the rest CAN be zero?
      if (webModel.pluggedIn) {
        webModelFunctions.behaviorStatusUpdate(
          'Make Map: Robot is still plugged in!',
        );
      } else {
        webModelFunctions.behaviorStatusUpdate(
          'Make Map: Robot is ready to make a map.',
        );
      }
      return true;
    }
    webModelFunctions.update('status', 'Make Map is starting...');
    return false;
  }
  if (webModel.ROSisRunning && webModel.makeMap) {
    robotModel.makeMap.start();
    webModelFunctions.behaviorStatusUpdate('Make Map');
    return false;
  }
  // This behavior is idle, allow behave loop to continue to next entry.
  return true;
}

module.exports = makeMap;
