// TODO: Check for CPU hogs, slow things, and maybe things that repeat too often:
//       https://www.npmjs.com/package/0x

const os = require('os');
const personalData = require('./personalData');
// We *DO* need webModel, don't remove it.
// eslint-disable-next-line no-unused-vars
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
const killROS = require('./killROS');

const handleSemaphoreFiles = require('./handleSemaphoreFiles');

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
    myCroft.init();
    setTimeout(() => {
      // Give init time to finish.
      tts(`Hello, my name is ${personalData.robotName}`);
    }, 3000);
  } else {
    tts(`Hello, my name is ${personalData.robotName}`);
  }

  // ## Scripts and ROS Commands that will be called by nodes ##
  // NOTE: Be sure to put 'unbuffer ' at the beginning of any ROScommand
  // if you hope to monitor the output, otherwise they never flush!
  // http://stackoverflow.com/a/11337310
  // http://linux.die.net/man/1/unbuffer

  robotModel.RosProcess = new LaunchScript({
    name: 'ROS',
    scriptName: '../scripts/start-robot.sh',
    successString: ']: Propellerbot_node has started.',
    // TODO: Could this be set to "true" by the ROSlibJS connection instead??
  });

  robotModel.goToWaypointProcess = new LaunchScript({
    name: 'GoToWaypoint',
    scriptName: '../scripts/gotoMapPositionHelperScript.sh',
    finalSuccessResultString: 'result: True',
    finalFailureResultString: 'result: False',
  });

  robotModel.makeMap = new LaunchScript({
    name: 'MakeMap',
    scriptName: '../scripts/make-map.sh',
    successString: 'odom received',
  });

  robotModel.loadMapProcess = new LaunchScript({
    name: 'LoadMap',
    scriptName: '../scripts/load-map.sh',
    successString: 'odom received',
  });

  handleSemaphoreFiles.startSemaphoreFileWatcher();

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
