const personalData = require('./personalData');
// eslint-disable-next-line no-unused-vars
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');

webModelFunctions.update('robotName', personalData.robotName);
const robotModel = require('./robotModel');
const UsbRelay = require('./UsbRelayControl');

robotModel.usbRelay = new UsbRelay();
const behave = require('./behave');
// Note that tts will convert text to speech,
// or it will send a ".wav" string (path, etc)
// to aplay.
// The benefit of using it is that it will honor
// system wide "bequiet" requests./usr/bin/upower -d|grep percentage|head -1
// The same module is used by ROS Python scripts.
const tts = require('./tts');
const LaunchScript = require('./LaunchScript');
const rosInterface = require('./rosInterface');
const webserver = require('./webserver');
const os = require('os');
const killROS = require('./killROS');

if (personalData.useBlueToothBeacon) {
  // eslint-disable-next-line no-unused-vars,global-require
  const bluetoothBeaconScanner = require('./BluetoothBeaconScanner');
}
const SocketServerSubscriber = require('./SocketServerSubscriber');
const RemoteMessageHandler = require('./RemoteMessageHandler');

const remoteMessageHandler = new RemoteMessageHandler();
const socketServerSubscriber = new SocketServerSubscriber(
  remoteMessageHandler.handleMessage,
);
socketServerSubscriber.start();

async function main() {
  rosInterface.start();

  // Cleanup on shutdown
  // http://stackoverflow.com/questions/14031763/doing-a-cleanup-action-just-before-node-js-exits

  function exitHandler(options, err) {
    if (options.cleanup) {
      console.log('Shutdown complete.');
      webModelFunctions.scrollingStatusUpdate('Shutdown complete.');
    }
    if (err) {
      console.log('Process Error:');
      console.log(err);
      console.log(err.stack);
      webModelFunctions.scrollingStatusUpdate(err.stack);
    }
    // if (options.exit) process.exit();
    if (options.exit) {
      webModelFunctions.scrollingStatusUpdate(
        'Main process exit, calling killROS',
      );
      console.log('Main process exit, calling killROS');
      killROS(true);
    }
  }

  // do something when app is closing
  process.on(
    'exit',
    exitHandler.bind(null, {
      cleanup: true,
    }),
  );

  // catches ctrl+c event
  process.on(
    'SIGINT',
    exitHandler.bind(null, {
      exit: true,
    }),
  );

  // catches uncaught exceptions
  process.on(
    'uncaughtException',
    exitHandler.bind(null, {
      exit: true,
    }),
  );

  webserver.start();

  if (personalData.useMyCroft) {
    // The purpose of using myCroft to simply announce the robot's name is to validate that MyCroft is up.
    // Also, If you use the robot's name as a wake word and send it via TTS, MyCroft hears it and tries to respond.
    // Otherwise the normal speech engine is the better option for event based announcements.
    // MyCroft is primarily for interactive operations, such as asking the robot to do things.
    // eslint-disable-next-line global-require
    const myCroft = require('./MyCroft');
    webModelFunctions.update('myCroftIsRunning', true);
    myCroft.init();
    setTimeout(() => {
      // Give init time to finish.
      myCroft.injectText(`arlobotstartupskill ${personalData.robotName}`);
    }, 3000);
  } else {
    tts(`Hello, my name is ${personalData.robotName}`);
  }

  // ## Scripts and ROS Commands that will be called by nodes ##
  // NOTE: Be sure to put 'unbuffer ' at the beginning of any ROScommand
  // if you hope to monitor the output, otherwise they never flush!
  // http://stackoverflow.com/a/11337310
  // http://linux.die.net/man/1/unbuffer

  robotModel.ROSprocess = new LaunchScript({
    name: 'ROS',
    scriptName: '../scripts/start-robot.sh',
    successString: ']: Propellerbot_node has started.',
    // TODO: Could this be set to "true" by the ROSlibJS connection instead??
  });

  /* GotoWaypoint Process output:
     Running GoToWaypoint child process . . .
     GoToWaypoint is starting up . . .
     GoToWaypoint stdout data:result: True
     GoToWaypoint exited with code: 0

     So just wait for a clean exit, no need for a successString.
     */
  robotModel.goToWaypointProcess = new LaunchScript({
    name: 'GoToWaypoint',
    // Set the ROScommmand at call time!
  });

  let exploreCommand;
  if (personalData.use_xv11) {
    exploreCommand =
      'unbuffer roslaunch arlobot_launchers add_autonomous_explore_xv11.launch';
  } else {
    exploreCommand =
      'unbuffer roslaunch arlobot_launchers add_autonomous_explore.launch';
  }
  robotModel.exploreProcess = new LaunchScript({
    name: 'Explore',
    ROScommand: exploreCommand,
    successString: 'odom received',
  });

  robotModel.makeMap = new LaunchScript({
    name: 'MakeMap',
    ROScommand: 'unbuffer roslaunch arlobot_navigation gmapping_demo.launch',
    successString: 'odom received',
  });

  let loadMapCommand;
  if (personalData.use_xv11) {
    loadMapCommand =
      'unbuffer roslaunch arlobot_launchers load_map_xv11.launch map_file:=';
  } else {
    loadMapCommand =
      'unbuffer roslaunch arlobot_launchers load_map.launch map_file:=';
  }
  robotModel.loadMapProcess = new LaunchScript({
    name: 'LoadMap',
    ROScommand: loadMapCommand,
    successString: 'odom received',
  });

  webModelFunctions.update('status', 'Starting behaviors.');

  // See behave.js for what robot "does" while it waits.
  // This is the main "loop" for the robot.
  behave();

  console.log(`Go to: http://${os.hostname()}:${personalData.webServerPort}/`);

  // Run Firefox with this page on the local machine,
  // to provide a "menu" on the robot!
  if (personalData.launchBrowser) {
    const runFirefox = new LaunchScript({
      name: 'FireFox',
      scriptName: '../scripts/runFirefox.sh',
      scriptArguments: `http://${os.hostname()}:${personalData.webServerPort}/`,
    });
    runFirefox.start();
  }
}

if (require.main === module) {
  (async () => {
    try {
      await main();
    } catch (e) {
      console.error('Robot failed with error:');
      console.error(e);
    }
  })();
}
