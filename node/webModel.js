// This object is sent to the web app via socket.io
// Do not put deep or complex objects here,
// Just text based or boolean items to digest in the web.
// Note that there are functions for working with this object,
// in webModelFunctions.js

module.exports = {
    robotName: 'ArloBot', // This will be overridden by personalData in index.js,
    myCroftIsRunning: false,
    myCroftSaid: '',
    lastUpdateTime: 0,
    videoSource: 'xscreen.png',
    masterRelayOn: false,
    debugging: false,
    cameraOn: false,
    ROSstart: false,
    ROSisRunning: false,
    killRosHasRun: false,
    pluggedIn: true, // Assume the most restrictive setting.
    autoExplore: false,
    pauseExplore: false,
    makeMap: false,
    makeMapRunning: false,
    idleTimeout: true,
    beQuiet: false,
    haltRobot: false,
    doorsOpen: true, // Track if any doors are open that pose a danger to Daleks
    semaphoreFilesRead: false,
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
    neoPixelsOn: false,
    relays: [],
    wayPoints: [],
    rosParameters: { // These are the parameters we will send to the web
        ignoreCliffSensors: false, // Use the expected default.
        ignoreProximity: false,
        ignoreIRSensors: false,
        ignoreFloorSensors: false,
        monitorACconnection: true,
        mapName: null,
        explorePaused: false
    },
    rosTopicItems: [
        // From ROS Topic /arlo_status
        {rosName: 'cliff', fancyName: 'Cliff Detected', status: '', btnClass: '', alertOn: true, alertBtnClass: 'btn-danger'},
        {rosName: 'floorObstacle', fancyName: 'Floor Obstacle', status: '', btnClass: '', alertOn: true, alertBtnClass: 'btn-danger'},
        {rosName: 'safeToRecede', fancyName: 'Safe To Recede', status: '', btnClass: '', alertOn: false, alertBtnClass: 'btn-danger'},
        {rosName: 'safeToProceed', fancyName: 'Safe To Proceed', status: '', btnClass: '', alertOn: false, alertBtnClass: 'btn-danger'},
        {rosName: 'Escaping', fancyName: 'Escaping', status: '', btnClass: '', alertOn: true, alertBtnClass: 'btn-danger'},
        {rosName: 'minDistanceSensor', fancyName: 'Min Distance Sensor', status: '', btnClass: '', alertOn: false, alertBtnClass: 'btn-danger'},
        {rosName: 'abd_speedLimit', fancyName: 'Forward Speed Limit', status: '', btnClass: '', alertOn: '<', alertValue: 100, alertBtnClass: 'btn-warning', specialCompare: true},
        {rosName: 'abdR_speedLimit', fancyName: 'Reverse Speed Limit', status: '', btnClass: '', alertOn: '<', alertValue: 100, alertBtnClass: 'btn-warning', specialCompare: true},
        {rosName: 'acPower', fancyName: 'AC Power', status: '', btnClass: '', alertOn: true, alertBtnClass: 'btn-danger'},
        {rosName: 'Heading', fancyName: 'Heading', status: '', btnClass: '', alertOn: false, alertBtnClass: 'btn-danger'},
        {rosName: 'gyroHeading', fancyName: 'Gyro Heading', status: '', btnClass: '', alertOn: false, alertBtnClass: 'btn-danger'},
        {rosName: 'leftMotorPower', fancyName: 'Left Motor Power', status: '', btnClass: '', alertOn: false, alertBtnClass: 'btn-danger'},
        {rosName: 'rightMotorPower', fancyName: 'Right Motor Power', status: '', btnClass: '', alertOn: false, alertBtnClass: 'btn-danger'},
        {rosName: 'laptopBatteryPercent', fancyName: 'Laptop Battery %', status: '', btnClass: '', alertOn: false, alertBtnClass: 'btn-danger'},
        {rosName: 'robotBatteryLevel', fancyName: 'Robot Battery Volts', status: '', btnClass: '', alertOn: '<', alertValue: 12, alertBtnClass: 'btn-danger'},
        {rosName: 'robotBatteryLow', fancyName: 'Robot Battery Low', status: '', btnClass: '', alertOn: true, alertBtnClass: 'btn-danger'}
    ],
    unplugYourself: false, // Indicates that robot should unplug itself.
    wayPointNavigator: {
        wayPointName: undefined,
        goToWaypoint: false, // This is set true when we ask the robot to go somewhere
        mostRecentArrival: undefined // Where we have most recently arrived at.
    }
};
