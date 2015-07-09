// This object is sent to the web app via socket.io
// Do not put deep or complex objects here,
// Just text based or boolean items to digest in the web.

module.exports = {
    robotName: 'ArloBot',
    debugging: false,
    ROSstart: false,
    ROSisRunning: false,
    pluggedIn: true, // Assume the most restrictive setting.
    autoExplore: false,
    pauseExplore: false,
    beQuiet: false,
    haltRobot: false,
    basementDoorOpen: true,
    laptopFullyCharged: 'unknown',
    laptopBatteryPercentage: '???%',
    logStreamerRunning: false,
    shutdownRequested: false,
    status: 'Arlo behavior is not running.',
    behaviorStatus: '',
    scrollingStatus: '',
    mapList: ['Explore!'],
    QRcode: '',
    mapName: '',
    wayPoints: [],
    rosParameters: { // These are the parameters we will send to the web
        ignoreCliffSensors: false, // Use the expected default.
        ignoreProximity: false,
        ignoreIRSensors: false,
        monitorACconnection: true
    }
};
