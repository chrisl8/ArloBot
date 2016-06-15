const personalData = require('./personalData');
const webModel = require('./webModel');
const robotModel = require('./robotModel');
const webModelFunctions = require('./webModelFunctions');
const EddystoneBeaconScanner = require('eddystone-beacon-scanner');
const inRoomDistance = 15;
const lostSignalTimeout = 120; // Seconds;
const leftRoomTimeout = 5; // Seconds
var possibleSignalLoss;
var possiblyLeftTheRoom;
const delayLostSignalResponse = function () {
    if (possiblyLeftTheRoom) {
        clearTimeout(possiblyLeftTheRoom);
        possiblyLeftTheRoom = undefined;
    }
    webModelFunctions.updateRobotMasterStatus('isInRoom', false);
    webModelFunctions.updateRobotMasterStatus('isClose', false);
    possibleSignalLoss = undefined;
};
const delayLeftRoomResponse = function () {
    webModelFunctions.updateRobotMasterStatus('isInRoom', false);
    possiblyLeftTheRoom = undefined;
};

var workingAverage;
var previousValue;
var threwOneOut = false;
EddystoneBeaconScanner.on('found', function (beacon) {
    if (possibleSignalLoss) {
        clearTimeout(possibleSignalLoss);
        possibleSignalLoss = undefined;
    }
    workingAverage = beacon.distance;
    previousValue = beacon.distance;
    // console.log('found Eddystone Beacon:\n', JSON.stringify(beacon, null, 2));
    console.log(`Beacon distance: ${Math.round(beacon.distance)} NEW`);
    webModelFunctions.updateRobotMasterStatus('isClose', true);
});

EddystoneBeaconScanner.on('updated', function (beacon) {
    // console.log('updated Eddystone Beacon:\n', JSON.stringify(beacon, null, 2));
    // TODO: Move the constants up in the program
    // Throw out single instance spikes.
    const spikeThreshold = 20;
    if (!threwOneOut && Math.abs(beacon.distance - previousValue) > spikeThreshold) {
        threwOneOut = true;
    } else {
        previousValue = beacon.distance;
        threwOneOut = false;
    }
    var newValue = previousValue;

    // Smooth out the changes
    // This must be below 1. The closer to 1 it is, the more it smooths out the number.
    const smoothingFactor = 0.20;
    workingAverage = (newValue * smoothingFactor) + ( workingAverage * ( 1.0 - smoothingFactor) );
    console.log(`Beacon distance: ${Math.round(workingAverage)} ${Math.round(beacon.distance)}`);
    if (workingAverage < inRoomDistance) {
        if (possiblyLeftTheRoom) {
            clearTimeout(possiblyLeftTheRoom);
            possiblyLeftTheRoom = undefined;
        }
        if (!robotModel.master.isInRoom) {
            webModelFunctions.updateRobotMasterStatus('isInRoom', true);
        }
    } else {
        if (!possiblyLeftTheRoom) {
            possiblyLeftTheRoom = setTimeout(delayLeftRoomResponse, leftRoomTimeout * 1000);
        }
    }
});

EddystoneBeaconScanner.on('lost', function (beacon) {
    // console.log('lost Eddystone beacon:\n', JSON.stringify(beacon, null, 2));
    console.log(`Beacon lost.`);
    if (!possibleSignalLoss) {
        possibleSignalLoss = setTimeout(delayLostSignalResponse, lostSignalTimeout * 1000);
    }
});

EddystoneBeaconScanner.startScanning(true);

/* Sample output:
 found Eddystone Beacon:
 {
 "txPower": -59,
 "url": "https://goo.gl/wl2NJt",
 "id": "ecb428720d14",
 "type": "url",
 "rssi": -86,
 "distance": 0.19952623149688797,
 "lastSeen": 1465710585429
 }
 updated Eddystone Beacon:
 {
 "txPower": -59,
 "url": "https://goo.gl/wl2NJt",
 "id": "ecb428720d14",
 "type": "url",
 "rssi": -86,
 "distance": 0.19952623149688797,
 "lastSeen": 1465710585429
 }
 updated Eddystone Beacon:
 {
 "txPower": -59,
 "url": "https://goo.gl/wl2NJt",
 "id": "ecb428720d14",
 "type": "url",
 "rssi": -91,
 "distance": 0.35481338923357547,
 "lastSeen": 1465710587440
 }
 found Eddystone Beacon:
 {
 "txPower": -59,
 "url": "https://goo.gl/wl2NJt",
 "id": "ecb428720d14",
 "type": "url",
 "rssi": -86,
 "distance": 0.19952623149688797,
 "lastSeen": 1465710585429
 }
 updated Eddystone Beacon:
 {
 "txPower": -59,
 "url": "https://goo.gl/wl2NJt",
 "id": "ecb428720d14",
 "type": "url",
 "rssi": -86,
 "distance": 0.19952623149688797,
 "lastSeen": 1465710585429
 }
 updated Eddystone Beacon:
 {
 "txPower": -59,
 "url": "https://goo.gl/wl2NJt",
 "id": "ecb428720d14",
 "type": "url",
 "rssi": -91,
 "distance": 0.35481338923357547,
 "lastSeen": 1465710587440
 }
 found Eddystone Beacon:
 {
 "txPower": -59,
 "url": "https://goo.gl/wl2NJt",
 "id": "ecb428720d14",
 "type": "url",
 "rssi": -86,
 "distance": 0.19952623149688797,
 "lastSeen": 1465710585429
 }
 updated Eddystone Beacon:
 {
 "txPower": -59,
 "url": "https://goo.gl/wl2NJt",
 "id": "ecb428720d14",
 "type": "url",
 "rssi": -86,
 "distance": 0.19952623149688797,
 "lastSeen": 1465710585429
 }
 updated Eddystone Beacon:
 {
 "txPower": -59,
 "url": "https://goo.gl/wl2NJt",
 "id": "ecb428720d14",
 "type": "url",
 "rssi": -91,
 "distance": 0.35481338923357547,
 "lastSeen": 1465710587440
 }

 */