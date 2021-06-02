const fs = require('fs');
const glob = require('glob');
const personalData = require('./personalData');
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');

const batteryLevelFileWildcard = '/sys/class/power_supply/BAT*/uevent';
const pluggedInFileWildcard = '/sys/class/power_supply/AC*/uevent';
let pluggedInFile;
let batteryLevelFile;

glob(pluggedInFileWildcard, (er, fileName) => {
  if (er) {
    console.error('Error finding AC status file.');
  } else {
    pluggedInFile = fileName[0];
  }
});

glob(batteryLevelFileWildcard, (er, fileName) => {
  if (er) {
    console.error('Error finding Battery status file.');
  } else {
    batteryLevelFile = fileName[0]; // TODO: Make battery number configurable?
  }
});

const checkBattery = (logIt) => {
  let batteryLevelFoundInROS = false;
  let pluggedInStatusFoundInROS = false;
  if (webModel.ROSisRunning) {
    // If ROS is running, use existing ROS Topic input data.
    const rosBatteryLevelTopicIndex = webModel.rosTopicItems.findIndex(
      (x) => x.rosName === 'laptop_battery_percent',
    );
    if (
      rosBatteryLevelTopicIndex > -1 &&
      Number.isInteger(webModel.rosTopicItems[rosBatteryLevelTopicIndex].status)
    ) {
      webModelFunctions.update(
        'laptopBatteryPercentage',
        webModel.rosTopicItems[rosBatteryLevelTopicIndex].status,
      );
      batteryLevelFoundInROS = true;
    }

    const rosAcPowerTopicIndex = webModel.rosTopicItems.findIndex(
      (x) => x.rosName === 'acPower',
    );
    if (
      rosAcPowerTopicIndex > -1 &&
      (webModel.rosTopicItems[rosAcPowerTopicIndex].status === true ||
        webModel.rosTopicItems[rosAcPowerTopicIndex].status === false)
    ) {
      webModelFunctions.update(
        'pluggedIn',
        webModel.rosTopicItems[rosAcPowerTopicIndex].status,
      );
      pluggedInStatusFoundInROS = true;
    }
  }

  // If ROS is not running, check Linux files for battery and plugged in state.
  if (!batteryLevelFoundInROS && batteryLevelFile) {
    fs.readFile(batteryLevelFile, 'utf8', (err, data) => {
      if (err) {
        console.error('Error getting battery level');
      } else {
        webModelFunctions.update(
          'laptopBatteryPercentage',
          parseInt(
            data
              .split('\n')
              .find((result) => result.indexOf('POWER_SUPPLY_CAPACITY') > -1)
              .split('=')[1],
            10,
          ),
        );
      }
    });
  }

  if (!pluggedInStatusFoundInROS && pluggedInFile) {
    fs.readFile(pluggedInFile, 'utf8', (err, data) => {
      if (err) {
        console.error('Error reading AC status file.');
      } else {
        webModelFunctions.update(
          'pluggedIn',
          data.split('\n').indexOf('POWER_SUPPLY_ONLINE=1') > -1,
        );
      }
    });
  }

  /** @namespace personalData.batteryConsideredFullAt */
  if (Number.isInteger(webModel.laptopBatteryPercentage)) {
    if (
      webModel.laptopBatteryPercentage >= personalData.batteryConsideredFullAt
    ) {
      webModelFunctions.update('laptopFullyCharged', true);
    } else {
      webModelFunctions.update('laptopFullyCharged', false);
    }
  }

  if (logIt) {
    console.log(
      webModel.laptopBatteryPercentage,
      webModel.pluggedIn,
      webModel.laptopFullyCharged,
    );
  }
};
module.exports = checkBattery;

if (require.main === module) {
  // Run the function if this is called directly instead of required.
  // It takes a few milliseconds for glob to get the battery file name.
  // To save processing time this is stored and reused.
  // In normal operation a "miss" early on doesn't matter, but if we run it from
  // the terminal, missing the first and only run is catastrophic, thus the timeout.
  setTimeout(() => {
    checkBattery(true);
  }, 50);
}
