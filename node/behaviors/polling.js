/*
 * This is a bit of a catchall.
 * Certainly some things here could be broken out into their own behavior files.
 * These are just things to do on every loop.
 */
const webModel = require('../webModel');
const webModelFunctions = require('../webModelFunctions');
const personalData = require('../personalData');
const robotModel = require('../robotModel');
const speechEngine = require('../speechEngine');
const checkBattery = require('../checkBattery');
const masterRelay = require('../MasterRelay');
const spawn = require('child_process').spawn;
const handleSemaphoreFiles = require('../handleSemaphoreFiles');
const getQRcodes = require('../getQRcodes');
const saveScreenShotForWeb = require('../saveScreenShotForWeb');
const howManySecondsSince = require('../howManySecondsSince');

const publishRobotURL = require('../publishRobotURL');

let intervalCount = 0; // Use this to throttle or space out polling items.
const intervalTop = 10;

async function polling() { // Argument options: tick
    intervalCount++;
    if (intervalCount > intervalTop) {
        intervalCount = 0;
    }
    if (webModel.debugging) {
        console.log('Polling');
        webModelFunctions.scrollingStatusUpdate('Polling', intervalCount);
    }
    /* Some things just need to be polled, there is no way around it. Put those here.
     This only repeats about once per second, so it is a pretty good spacing, even without fancy code to slow things down
     but if you want to , set it to only happen when interval === some number
     below intervalTop.
     Or use some division to hit every odd/even number or multiple of 3.
     */

    speechEngine();

    handleSemaphoreFiles.readSemaphoreFiles();
    if (intervalCount === 5) {
        publishRobotURL.updateRobotURL();
    }
    if ((intervalCount % 2) === 0) {
        checkBattery();
    }
    if ((intervalCount % 2) === 1) {
        masterRelay('read');
        robotModel.usbRelay.updateAllRelayState();
    }
    if (intervalCount === intervalTop) {
        if (!webModel.cameraOn) {
            saveScreenShotForWeb();
        }
    }

    // Check for QR code:
    // If ROS has started, only do this when idle, but before ROS starts we can do it also,
    // that way it can have the map BEFORE ROS starts!
    // And also it won't text me with "Where am I?" if it is sitting in front of a QR code.
    // NOTE: At this point, once it gets an "unplug yourself" or "ROSstart" = true,
    // It will stop polling for QR codes.
    // But if we want to look for others later, remove "!webModel.hasSetupViaQRcode",
    // and it still will not set those two again (due to code in getQRcodes),
    // but it may fill in a map or fill in the webModel.QRcode line.
    // This may cause it to fight with other camera operations though.
    /** @namespace personalData.useQRcodes */
    if (intervalCount === 9 && !webModel.hasSetupViaQRcode && personalData.useQRcodes && !robotModel.gettingQRcode && !webModel.killRosHasRun && (robotModel.cmdTopicIdle || !webModel.ROSstart)) {
        // Old school thread control
        // It reduces how often zbarcam is run,
        // and prevents it from getting stuck
        robotModel.gettingQRcode = true;
        // TODO: This doesn't work with the Master & 5volt relays off!
        getQRcodes();
    }

    // If we are not finding a QRcode and no map is listed,
    // try turning on the light for a minute to see if it helps.
    // NOTE: Right now it won't do this if ROSstart is true,
    // assuming that if we started it manually, we don't want it to look
    // for a QR code for a map by itself
    const tryLightDelayTime = 60 * 2; // Two minutes
    if (!webModel.ROSstart && !webModel.hasSetupViaQRcode && !webModel.triedLightToFindQRcode && webModel.mapName === '' && howManySecondsSince(robotModel.bootTime) >= tryLightDelayTime && personalData.useQRcodes && !webModel.killRosHasRun) {
        webModelFunctions.update('triedLightToFindQRcode', true);
        spawn('../scripts/turn_on_light.sh');
        setTimeout(function () {
            if (!webModel.userLightOnRequested) {
                spawn('../scripts/turn_off_light.sh');
            }
        }, 60);
    }

    // Idle timer to shut off robot when left unattended
    // NOTE: The handlePowerWithoutROS behavior deals with power when ROS isn't running.
    if (webModel.ROSisRunning && webModel.idleTimeout && personalData.idleTimeoutInMinutes > 0) {
        // Set to now to fake out idle timer if no action is required.
        const dateNow = new Date();

        // When ROS is active the idle timer is tied to the twist message command topic.
        if (robotModel.cmdTopicIdle) {
            const lastActionDate = new Date(robotModel.lastMovementTime);
            const idleMinutes = (dateNow - lastActionDate) / 1000 / 60;
            if (webModel.debugging) {
                console.log(`ROS Idle Check: ${dateNow} - ${lastActionDate} = ${idleMinutes}`);
            }
            if (idleMinutes > personalData.idleTimeoutInMinutes) {
                console.log("ROS Idle shutdown.");
                webModelFunctions.scrollingStatusUpdate("ROS Idle shutdown.");
                webModelFunctions.update('shutdownRequested', true);
            }
        }
    } else if (webModel.debugging) {
        console.log(`ROS Idle Check: Robot not idle.`);
    }

    // After 2 hours be quiet, and turn on the idle timer again.
    // NOTE: If you turn on debugging, this will update every behavior loop (1 second)
    const dateNow = new Date();
    const lastActionDate = new Date(webModel.lastUpdateTime);
    const idleMinutes = (dateNow - lastActionDate) / 1000 / 60;
    if (idleMinutes > 120) {
        handleSemaphoreFiles.setSemaphoreFiles('beQuiet');
        webModelFunctions.update('idleTimeout', true);
    }


    /* TODO:
     2. Set up some sort of idle warning:
     a. Talk
     b. Web interface popup.
     */

    // This behavior will always return success,
    return true;
}

module.exports = polling;
