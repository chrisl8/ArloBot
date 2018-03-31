const webModel = require('../webModel');
const webModelFunctions = require('../webModelFunctions');
const rosInterface = require('../rosInterface');
const robotModel = require('../robotModel');

async function autoExplore() {
    if (webModel.debugging) {
        console.log('Auto Explore');
        webModelFunctions.scrollingStatusUpdate('Auto Explore');
    }
    if (robotModel.exploreProcess.started) {

        // Catch changes in pauseExplore and send them to the arlobot_explore pause_explorer service
        if (webModel.rosParameters.explorePaused !== webModel.pauseExplore) {
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
                webModelFunctions.behaviorStatusUpdate('Auto Explore: FAILURE');
                return false;
            } else {
                webModelFunctions.update('status', 'Explore process started.');
                if (webModel.pluggedIn) {
                    webModelFunctions.behaviorStatusUpdate('Auto Explore: Robot is still plugged in!');
                } else {
                    webModelFunctions.behaviorStatusUpdate('Auto Explore: Robot is Exploring!');
                }
                return true;
            }
        } else {
            webModelFunctions.update('status', 'Explore process is starting...');
            return false;
        }
    } else {
        robotModel.exploreProcess.start();
        webModelFunctions.behaviorStatusUpdate('Auto Explore');
        return false;
    }
}

module.exports = autoExplore;
