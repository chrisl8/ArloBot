/*
 * This is a bit of a catchall.
 * Certainly some things here could be broken out into their own behavior files.
 * These are just things to do on every loop.
 */
const spawn = require('child_process').spawn;
const webModel = require('../webModel');
const webModelFunctions = require('../webModelFunctions');
const personalData = require('../personalData');
const robotModel = require('../robotModel');
const speechEngine = require('../speechEngine');
const checkBattery = require('../checkBattery');
const masterRelay = require('../MasterRelay');
const getQRcodes = require('../getQRcodes');
const saveScreenShotForWeb = require('../saveScreenShotForWeb');
const howManySecondsSince = require('../howManySecondsSince');
const getCmdVelIdleTime = require('../getCmdVelIdleTime');

const publishRobotURL = require('../publishRobotURL');

let intervalCount = 0; // Use this to throttle or space out polling items.
const intervalTop = 10; // Seconds

async function polling() {
  // Argument options: tick
  intervalCount++;
  if (intervalCount > intervalTop) {
    intervalCount = 0;
  }
  if (webModel.debugging && webModel.logBehaviorMessages) {
    const message = ' - Polling';
    console.log(message);
    webModelFunctions.scrollingStatusUpdate(message);
  }
  /* Some things just need to be polled, there is no way around it. Put those here.
   This only repeats about once per second, so it is a pretty good spacing, even without fancy code to slow things down
   but if you want to slow it down, set it to only happen when interval === some number
   below intervalTop.
   Or use some division to hit every odd/even number or multiple of 3.
   */

  speechEngine();

  if (intervalCount === 5) {
    // seconds
    // noinspection ES6MissingAwait
    publishRobotURL.updateRobotURL();
  }
  if (intervalCount === 0) {
    checkBattery();
  }
  if (intervalCount === intervalTop) {
    if (!webModel.cameraOn) {
      saveScreenShotForWeb();
    }
  }
  if (webModel.checkMasterRelay) {
    webModelFunctions.update('checkMasterRelay', false);
    masterRelay('read');
  }
  if (webModel.checkUsbRelayBank) {
    webModelFunctions.update('checkUsbRelayBank', false);
    robotModel.usbRelay.updateAllRelayState();
  }

  // Check for QR code:
  // This only runs before ROS is started.
  // That way it can have the map BEFORE ROS starts.
  // If we want it to run after starting ROS, it needs to
  // check and compare the robotModel.lastMovementTime
  /** @namespace personalData.useQRcodes */
  if (
    !webModel.ROSstart &&
    intervalCount === 9 &&
    !webModel.hasSetupViaQRcode &&
    personalData.useQRcodes &&
    !robotModel.gettingQrCode &&
    !webModel.killRosHasRun
  ) {
    // Old school thread control
    // It reduces how often zbarcam is run,
    // and prevents it from getting stuck
    robotModel.gettingQrCode = true;
    // TODO: This doesn't work with the Master & 5volt relays off!
    getQRcodes();
  }

  // If we are not finding a QR code and no map is listed,
  // try turning on the light for a minute to see if it helps.
  // NOTE: Right now it won't do this if ROSstart is true,
  // assuming that if we started it manually, we don't want it to look
  // for a QR code for a map by itself
  const tryLightDelayTime = 60 * 2; // Two minutes
  if (
    !webModel.ROSstart &&
    !webModel.hasSetupViaQRcode &&
    !webModel.triedLightToFindQRcode &&
    webModel.mapName === '' &&
    howManySecondsSince(robotModel.bootTime) >= tryLightDelayTime &&
    personalData.useQRcodes &&
    !webModel.killRosHasRun
  ) {
    webModelFunctions.update('triedLightToFindQRcode', true);
    spawn('../scripts/turn_on_light.sh');
    setTimeout(() => {
      if (!webModel.userLightOnRequested) {
        spawn('../scripts/turn_off_light.sh');
      }
    }, 60);
  }

  if (webModel.debugging && webModel.logBehaviorMessages) {
    console.log(`   - Idle Minutes: ${getCmdVelIdleTime()}`);
  }

  // Idle timer to shut off robot when left unattended
  // NOTE: The handlePowerWithoutROS behavior deals with power when ROS isn't running.
  if (
    webModel.ROSisRunning &&
    webModel.idleTimeout &&
    personalData.idleTimeoutInMinutes > 0 &&
    getCmdVelIdleTime() > personalData.idleTimeoutInMinutes
  ) {
    console.log('ROS Idle shutdown.');
    webModelFunctions.scrollingStatusUpdate('ROS Idle shutdown.');
    webModelFunctions.update('shutdownRequested', true);
  }

  // After 2 hours turn on the idle timer again.
  const dateNow = new Date();
  const lastActionDate = new Date(webModel.lastUpdateTime);
  const idleMinutes = (dateNow - lastActionDate) / 1000 / 60;
  if (idleMinutes > 120) {
    if (webModel.debugging && webModel.logBehaviorMessages) {
      console.log(
        `   - Enabling Idle Timeout again after 2 hours of inactivity.`,
      );
    }
    webModelFunctions.update('idleTimeout', true);
  }

  // This behavior is idle, allow behave loop to continue to next entry.
  return true;
}

module.exports = polling;
