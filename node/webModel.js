// This object is sent to the web app via socket.io
// Do not put deep or complex objects here,
// Just text based or boolean items to digest in the web.
// Note that there are functions for working with this object,
// in webModelFunctions.js

module.exports = {
  robotName: 'ArloBot', // This will be overridden by personalData in index.js,
  cloudServerConnected: false,
  myCroftIsRunning: false,
  myCroftSaid: '',
  lastUpdateTime: 0,
  videoSource: 'xscreen.png',
  masterRelayOn: false,
  debugging: false,
  logConsoleMessages: false, // Like debugging but ONLY for LaunchScript function
  logBehaviorMessages: false,
  logOtherMessages: false,
  logTalkAboutEvents: false,
  cameraOn: false,
  ROSstart: false,
  ROSisRunning: false,
  killRosHasRun: false,
  pluggedIn: true, // Assume the most restrictive setting.
  makeMap: false,
  makeMapRunning: false,
  idleTimeout: true,
  beQuiet: false,
  checkMasterRelay: true, // Always read once on load
  checkUsbRelayBank: true, // Always read once on load
  haltRobot: false,
  semaphoreFilesRead: false,
  laptopFullyCharged: 'unknown',
  laptopBatteryPercentage: '???',
  logStreamerRunning: false,
  shutdownRequested: false,
  status: 'Arlo behavior is not running.',
  behaviorStatus: '',
  scrollingStatus: [],
  mapList: [],
  QRcode: '',
  hasSetupViaQRcode: false, // So we only do this once. ;)
  mapName: '',
  mapLoaded: false,
  loadMapRequested: false,
  triedLightToFindQRcode: false,
  userLightOnRequested: false,
  robotIP: undefined,
  robotURL: undefined,
  neoPixelsOn: false,
  relays: [],
  wayPoints: [],
  rosParameters: {
    // These are the parameters we will send to the web
    ignoreCliffSensors: false, // Use the expected default.
    ignoreProximity: false,
    ignoreIRSensors: false,
    ignoreFloorSensors: false,
    mapName: '',
  },
  rosTopicItems: [
    // From ROS Topic /arlo_status
    {
      rosName: 'cliff',
      fancyName: 'Cliff Detected',
      status: '',
      btnClass: '',
      alertOn: true,
      alertBtnClass: 'btn-danger',
    },
    {
      rosName: 'floor_obstacle',
      fancyName: 'Floor Obstacle',
      status: '',
      btnClass: '',
      alertOn: true,
      alertBtnClass: 'btn-danger',
    },
    {
      rosName: 'safe_to_recede',
      fancyName: 'Safe To Recede',
      status: '',
      btnClass: '',
      alertOn: false,
      alertBtnClass: 'btn-danger',
    },
    {
      rosName: 'safe_to_proceed',
      fancyName: 'Safe To Proceed',
      status: '',
      btnClass: '',
      alertOn: false,
      alertBtnClass: 'btn-danger',
    },
    {
      rosName: 'escaping',
      fancyName: 'Escaping',
      status: '',
      btnClass: '',
      alertOn: true,
      alertBtnClass: 'btn-danger',
    },
    {
      rosName: 'min_distance_sensor',
      fancyName: 'Min Distance Sensor',
      status: '',
      btnClass: '',
      alertOn: false,
      alertBtnClass: 'btn-danger',
    },
    {
      rosName: 'abd_speed_limit',
      fancyName: 'Forward Speed Limit',
      status: '',
      btnClass: '',
      alertOn: '<',
      alertValue: 100,
      alertBtnClass: 'btn-warning',
      specialCompare: true,
    },
    {
      rosName: 'abd_reverse_speed_limit',
      fancyName: 'Reverse Speed Limit',
      status: '',
      btnClass: '',
      alertOn: '<',
      alertValue: 100,
      alertBtnClass: 'btn-warning',
      specialCompare: true,
    },
    {
      rosName: 'acPower',
      fancyName: 'AC Power',
      status: '',
      btnClass: '',
      alertOn: true,
      alertBtnClass: 'btn-danger',
    },
    {
      rosName: 'heading',
      fancyName: 'Heading',
      status: '',
      btnClass: '',
      alertOn: false,
      alertBtnClass: 'btn-danger',
    },
    {
      rosName: 'gyro_heading',
      fancyName: 'Gyro Heading',
      status: '',
      btnClass: '',
      alertOn: false,
      alertBtnClass: 'btn-danger',
    },
    {
      rosName: 'left_motor_power',
      fancyName: 'Left Motor Power',
      status: '',
      btnClass: '',
      alertOn: false,
      alertBtnClass: 'btn-danger',
    },
    {
      rosName: 'right_motor_power',
      fancyName: 'Right Motor Power',
      status: '',
      btnClass: '',
      alertOn: false,
      alertBtnClass: 'btn-danger',
    },
    {
      rosName: 'laptop_battery_percent',
      fancyName: 'Laptop Battery %',
      status: '',
      btnClass: '',
      alertOn: false,
      alertBtnClass: 'btn-danger',
    },
    {
      rosName: 'robot_battery_level',
      fancyName: 'Robot Battery Volts',
      status: '',
      btnClass: '',
      alertOn: '<',
      alertValue: 11.8,
      alertBtnClass: 'btn-danger',
    },
    {
      rosName: 'robot_battery_low',
      fancyName: 'Robot Battery Low',
      status: '',
      btnClass: '',
      alertOn: true,
      alertBtnClass: 'btn-danger',
    },
    {
      rosName: 'dangerous_doors_open',
      fancyName: 'Doors Open',
      status: '',
      btnClass: '',
      alertOn: true,
      alertBtnClass: 'btn-danger',
    },
    // Move base goals
    {
      rosName: 'goalStatus',
      fancyName: 'Goal Status',
      status: '',
      btnClass: '',
      alertOn: true,
      alertBtnClass: 'btn-danger',
    },
  ],
  unplugYourself: false, // Indicates that robot should unplug itself.
  wayPointNavigator: {
    wayPointName: null,
    goToWaypoint: false, // This is set true when we ask the robot to go somewhere
    mostRecentArrival: undefined, // Where we have most recently arrived at.
  },
  navigationInProgress: false,
  lastNavigationResult: false,
};
