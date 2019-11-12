const webModel = require('../webModel');
const webModelFunctions = require('../webModelFunctions');
const robotModel = require('../robotModel');

async function makeMapCartographer() {
  if (webModel.debugging) {
    console.log('Make Map Cartographer');
    webModelFunctions.scrollingStatusUpdate('Make Map Cartographer');
  }
  if (robotModel.makeMapCartographer.started) {
    if (robotModel.makeMapCartographer.startupComplete) {
      if (robotModel.makeMapCartographer.hasExited) {
        webModelFunctions.update(
          'status',
          'Make Map Cartographer process is closed.',
        );
        webModelFunctions.update('makeMapRunning', false); // NOTE: Shared with all Make Map behaviors
        webModelFunctions.update('makeMapCartographer', false);
        robotModel.makeMapCartographer.started = false;
        webModelFunctions.behaviorStatusUpdate(
          'Make Map Cartographer: FAILURE',
        );
        return false;
      }
      webModelFunctions.update(
        'status',
        'Make Map Cartographer process started.',
      );
      webModelFunctions.update('makeMapRunning', true); // NOTE: Shared with all Make Map behaviors
      if (webModel.pluggedIn) {
        webModelFunctions.behaviorStatusUpdate(
          'Make Map Cartographer: Robot is still plugged in!',
        );
      } else {
        webModelFunctions.behaviorStatusUpdate(
          'Make Map Cartographer: Robot is ready to make a map.',
        );
      }
      return true;
    }
    webModelFunctions.update('status', 'Make Map Cartographer is starting...');
    return false;
  }
  robotModel.makeMapCartographer.start();
  webModelFunctions.behaviorStatusUpdate('Make Map Cartographer');
  return false;
}

module.exports = makeMapCartographer;
