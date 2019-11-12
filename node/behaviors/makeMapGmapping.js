const webModel = require('../webModel');
const webModelFunctions = require('../webModelFunctions');
const robotModel = require('../robotModel');

async function makeMapGmapping() {
  if (webModel.debugging) {
    console.log('Make Map Gmapping');
    webModelFunctions.scrollingStatusUpdate('Make Map Gmapping');
  }
  if (robotModel.makeMapGmapping.started) {
    if (robotModel.makeMapGmapping.startupComplete) {
      if (robotModel.makeMapGmapping.hasExited) {
        webModelFunctions.update(
          'status',
          'Make Map Gmapping process is closed.',
        );
        webModelFunctions.update('makeMapRunning', false); // NOTE: Shared with all Make Map behaviors
        webModelFunctions.update('makeMapGmapping', false);
        robotModel.makeMapGmapping.started = false;
        webModelFunctions.behaviorStatusUpdate('Make Map Gmapping: FAILURE');
        return false;
      }
      webModelFunctions.update('status', 'Make Map Gmapping process started.');
      webModelFunctions.update('makeMapRunning', true); // NOTE: Shared with all Make Map behaviors
      if (webModel.pluggedIn) {
        webModelFunctions.behaviorStatusUpdate(
          'Make Map Gmapping: Robot is still plugged in!',
        );
      } else {
        webModelFunctions.behaviorStatusUpdate(
          'Make Map Gmapping: Robot is ready to make a map.',
        );
      }
      return true;
    }
    webModelFunctions.update('status', 'Make Map Gmapping is starting...');
    return false;
  }
  robotModel.makeMapGmapping.start();
  webModelFunctions.behaviorStatusUpdate('Make Map Gmapping');
  return false;
}

module.exports = makeMapGmapping;
