'use strict';
const personalData = require('./personalData');
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
const fs = require('fs');
const glob = require('glob');
const batteryLevelFileWildcard = '/sys/class/power_supply/BAT*/uevent';
const pluggedInFileWildcard = '/sys/class/power_supply/AC*/uevent';
var pluggedInFile, batteryLevelFile;

glob(pluggedInFileWildcard, function (er, fileName) {
    if (er) {
        console.error('Error finding AC status file.');
    } else {
        pluggedInFile = fileName[0];
    }
});

glob(batteryLevelFileWildcard, function (er, fileName) {
    if (er) {
        console.error('Error finding Battery status file.');
    } else {
        batteryLevelFile = fileName[0]; // TODO: Make battery number configurable?
    }
});

const checkBattery = function (logIt) {
    if (batteryLevelFile) {
        fs.readFile(batteryLevelFile, 'utf8', function (err, data) {
            if (err) {
                console.error('Error getting battery level');
            } else {
                webModelFunctions.update('laptopBatteryPercentage', parseInt(data.split('\n').find(function (data) {
                    return data.indexOf('POWER_SUPPLY_CAPACITY') > -1;
                }).split('=')[1]));
                if (webModel.laptopBatteryPercentage >= personalData.batteryConsideredFullAt) {
                    webModelFunctions.update('laptopFullyCharged', true);
                } else {
                    webModelFunctions.update('laptopFullyCharged', false);
                }
                if (logIt) {
                    console.log(webModel.laptopBatteryPercentage, webModel.pluggedIn, webModel.laptopFullyCharged);
                }
            }
        });
    }

    if (pluggedInFile) {
        fs.readFile(pluggedInFile, 'utf8', function (err, data) {
            if (err) {
                console.error('Error reading AC status file.');
            } else {
                webModelFunctions.update('pluggedIn', data.split('\n').indexOf('POWER_SUPPLY_ONLINE=1') > -1)
            }
        });
    }
};
module.exports = checkBattery;

if (require.main === module) {
    // Run the function if this is called directly instead of required.
    // It takes a few millisceonds for glob to get the batter file name.
    // To save processing time this is stored and reused.
    // In normal operation a "miss" early on doesn't matter, but if we run it from
    // the terminal, missing the first and only run is catastrophic, thus the timeout.
    setTimeout(function () {
        checkBattery(true);
    }, 50);
}
