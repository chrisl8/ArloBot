const personalData = require('../personalData');
const webModel = require('../webModel');
const webModelFunctions = require('../webModelFunctions');
const robotModel = require('../robotModel');
const masterRelay = require('../MasterRelay');
const wait = require('../wait');

async function handleUsbHubPower() {
  if (webModel.debugging) {
    console.log('Handle Power without ROS');
    webModelFunctions.scrollingStatusUpdate('Handle Power without ROS');
  }

  // Power off the Master Relay and all regular Relays after the idle timeout,
  // whether plugged in or not.
  // NOTE: My last try at running this "continuously" it drained the 12v batteries,
  // even when plugged in. Maybe my batteries are old, but it seems best to NOT do that.
  // Hence, idle power off, even when plugged in.
  // NOTE2: The idle timeout for while ROS is running is handled in polling right now.
  if (
    webModel.idleTimeout &&
    personalData.idleTimeoutInMinutes > 0 &&
    webModel.masterRelayOn
  ) {
    // NOTE: If you turn on debugging, this will update every behavior loop (1 second)
    const dateNow = new Date();
    const lastActionDate = new Date(webModel.lastUpdateTime);

    const idleMinutes = (dateNow - lastActionDate) / 1000 / 60;
    if (webModel.debugging) {
      console.log(
        `Idle Check: ${dateNow} - ${lastActionDate} = ${idleMinutes}`,
      );
    }
    if (idleMinutes > personalData.idleTimeoutInMinutes) {
      console.log('Idle power down.');
      webModelFunctions.scrollingStatusUpdate('Idle power down.');
      if (personalData.useMasterPowerRelay) {
        await wait(1); // The usbRelay calls just made by polling can clobber this if we don't wait.
        masterRelay('off');
      }
      if (personalData.useUSBrelay) {
        await wait(1); // The usbRelay calls just made by polling can clobber this if we don't wait.
        robotModel.usbRelay.switchRelay('all', 'off');
      }
    }
  }
}

module.exports = handleUsbHubPower;
