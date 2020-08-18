const webModel = require('../webModel');
const webModelFunctions = require('../webModelFunctions');
const robotModel = require('../robotModel');

async function makeMap() {
  if (webModel.debugging) {
    console.log('Make Map');
    webModelFunctions.scrollingStatusUpdate('Make Map');
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
  robotModel.makeMap.start();
  webModelFunctions.behaviorStatusUpdate('Make Map');
  return false;
}

module.exports = makeMap;
