// This object is sent to the web app via socket.io
// Do not put deep or complex objects here,
// Just text based or boolean items to digest in the web.
// Note that there are functions for working with this object,
// in webModelFunctions.js

module.exports = {
    robotName: 'ArloBot', // This will be overridden by personalData in index.js,
    lastUpdateTime: 0,
    videoSource: 'xscreen.png',
    masterRelayOn: false,
    debugging: false,
    cameraOn: false,
    ROSstart: false,
    ROSisRunning: false,
    pluggedIn: true, // Assume the most restrictive setting.
    autoExplore: false,
    pauseExplore: false,
    beQuiet: false,
    haltRobot: false,
    doorsOpen: true, // Track if any doors are open that pose a danger to Daleks
    laptopFullyCharged: 'unknown',
    laptopBatteryPercentage: '???%',
    logStreamerRunning: false,
    colorFollowerRunning: false,
    shutdownRequested: false,
    status: 'Arlo behavior is not running.',
    behaviorStatus: '',
    scrollingStatus: '',
    mapList: ['Explore!'],
    QRcode: '',
    hasSetupViaQRcode: false, // So we only do this once. ;)
    mapName: '',
    triedLightToFindQRcode: false,
    userLightOnRequested: false,
    robotIP: undefined,
    robotURL: undefined,
    relays: [],
    wayPoints: [],
    rosParameters: { // These are the parameters we will send to the web
        ignoreCliffSensors: false, // Use the expected default.
        ignoreProximity: false,
        ignoreIRSensors: false,
        ignoreFloorSensors: false,
        monitorACconnection: true,
        mapName: null,
        explorePaused: false,
        // From ROS Topic /arlo_status
        floorObstacle: false,
        safeToRecede: true,
        safeToProceed: true,
        Escaping: false,
        acPower: true,
        gyroHeading: 0,
        rightMotorPower: false,
        minDistanceSensor: 3,
        abd_speedLimit: 10,
        abdR_speedLimit: 100,
        leftMotorPower: false,
        laptopBatteryPercent: 0,
        robotBatteryLow: false,
        Heading: 0,
        robotBatteryLevel: 0,
        cliff: false
    },
    unplugYourself: false, // Indicates that robot should unplug itself.
    wayPointNavigator: {
        wayPointName: undefined,
        goToWaypoint: false, // This is set true when we ask the robot to go somewhere
        mostRecentArrival: undefined // Where we have most recently arrived at.
    }
};
