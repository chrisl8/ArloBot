// This is for things that don't belong in webModel,
// or things that need to be "picked up" from weModel,
// and acted on before being tagged on the robot

module.exports = {
    debug: false,
    // Some timestamps for use to help know when to run stuff
    bootTime: new Date(), // Time the node script was initialized (Can I do this?)
    startROSTime: undefined, // Time that ROS start was completed.
    mapLoadTime: undefined,// Time that map load was complete
    makeMap: undefined,
    master: {
        isClose: false, // Eddy Beackon === 1
        isInRoom: false, // Eddy Beacon < 1
        isAsleep: false
    },
    whereamiTextSent: false,
    unplugMeTextSent: false,
    fullyCharged: false,
    pauseExplore: false,
    webCamInUse: false,
    gettingQRcode: false,
    initialPoseSet: false,
    cmdTopicIdle: false,
    lastMovementTime: 0,
    active_cmd: 'idle', // Hold the active cmd that is currently driving the robot.
    wayPointNavigator: {
        destinaitonWaypoint: undefined // This will be the place we want the robot to go to.
    }
};
