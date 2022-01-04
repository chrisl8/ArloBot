const fs = require('fs');
const mkdirp = require('mkdirp');
const express = require('express');
const _ = require('lodash');
const spawn = require('child_process').spawn;
const bodyParser = require('body-parser');
const socketIo = require('socket.io');

const personalData = require('./personalData');
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
const Camera = require('./Camera');
/** @namespace personalData.camera0name */
const camera = new Camera('Camera', personalData.camera0name);
const Arduino = require('./Arduino');

const arduino = new Arduino();
const robotModel = require('./robotModel');
const LaunchScript = require('./LaunchScript');
const tts = require('./tts');
const myCroft = require('./MyCroft');
const WayPoints = require('./WayPoints.js');

const wayPointEditor = new WayPoints();
const rosInterface = require('./rosInterface');
const masterRelay = require('./MasterRelay');
const updateMapList = require('./updateMapList');
const LCD = require('./LCD');

const personalDataFolder = `${process.env.HOME}/.arlobot/`;

LCD({ operation: 'color', red: 0, green: 0, blue: 255 });
LCD({ operation: 'clear' });
LCD({ operation: 'contrast', input: 200 });
LCD({
  operation: 'text',
  input: '-Server Running-',
  row: 'top',
});
LCD({
  operation: 'text',
  input: 'Initial Startup.',
  row: 'bottom',
});

updateMapList();

const app = express();

// For json encoded post requests, which I use:
app.use(bodyParser.json());
// Required for Twilio:
app.use(
  bodyParser.urlencoded({
    extended: true,
  }),
);
// var path = require('path');
// app.get('/', isLoggedIn, function (req, res) {
//     console.log('get /');
//     res.sendFile(path.join(__dirname + '/../website/index.html'));
// });

// All web content is housed in the website folder
app.use(express.static(`${__dirname}/../website/build`));

const handleSemaphoreFiles = require('./handleSemaphoreFiles');

const saveMapMakingGoals = async (newMapName, position, name) => {
  const waypointFolder = `${personalDataFolder}mapMakingGoals/${newMapName}/`;
  const wayPointFile = waypointFolder + name;
  await mkdirp(waypointFolder, 0o777);
  fs.writeFile(wayPointFile, position, (e) => {
    if (e) {
      webModelFunctions.scrollingStatusUpdate(
        `ERROR writing map making goal ${name} to disk`,
      );
      console.error(`ERROR writing map making goal ${name} to disk:`);
      console.error(e);
    }
    webModelFunctions.scrollingStatusUpdate(
      `Map making point ${name} has been written to disk`,
    );
  });
};

const saveMap = async (newMapName) => {
  // Text from service when it is finished is:
  // **Finished serializing Dataset**
  const saveMapProcess = new LaunchScript({
    name: 'SaveMap',
    callback: updateMapList,
    ROScommand: `${__dirname}/../scripts/save-map.sh ${newMapName}`,
    scriptArguments: newMapName,
  });
  saveMapProcess.start();
  for (let i = 0; i < robotModel.mapMakingGoalList.length; i++) {
    // eslint-disable-next-line no-await-in-loop
    await saveMapMakingGoals(newMapName, robotModel.mapMakingGoalList[i], i);
  }
};

const startLogStreamer = function () {
  const command = `${__dirname}/../scripts/log-watcher.sh`;
  const logStreamerProcess = spawn(command);
  logStreamerProcess.on('exit', (code) => {
    // Will catch multiple exit codes I think:
    if (code === 0) {
      webModelFunctions.scrollingStatusUpdate('Log streamer started');
      webModelFunctions.update('logStreamerRunning', true);
    } else {
      console.log(`Log streamer failed with code: ${code}`);
      webModelFunctions.update('logStreamerRunning', false);
    }
  });
  return logStreamerProcess;
};
const stopLogStreamer = function () {
  const command = '/usr/bin/pkill';
  const commandArgs = ['-f', 'log.io'];
  const child = spawn(command, commandArgs);
  child.on('exit', () => {
    webModelFunctions.scrollingStatusUpdate('Log streamer killed');
    webModelFunctions.update('logStreamerRunning', false);
  });
  return child;
};

async function start() {
  /** @namespace personalData.webServerPort */
  const webServer = app.listen(personalData.webServerPort);
  const io = socketIo(webServer, {
    cors: {
      origin: 'http://localhost:3000',
      methods: ['GET', 'POST'],
    },
  });
  // NOTE: The cors object is only actually used/needed for testing the site when running it remotely.
  // For production, it isn't used because we connect back to the same IP address
  // that the server runs on!

  const throttledWebModelEmitter = _.throttle(
    () => {
      io.sockets.emit('webModel', webModel);
    },
    personalData.socketEmitterThrottle
      ? personalData.socketEmitterThrottle
      : 500,
  );

  webModelFunctions.emitter.on('change', () => {
    throttledWebModelEmitter();
  });

  // Socket listeners
  io.on('connection', (socket) => {
    socket.emit('startup', webModel);
    const address = socket.request.connection.remoteAddress;
    console.log(`Web connection from ${address}`);

    socket.on('setMap', (data) => {
      if (data) {
        if (webModel.mapList.indexOf(data) > -1) {
          webModelFunctions.update('loadMapRequested', true);
          webModelFunctions.update('mapName', data);
          wayPointEditor.updateWayPointList();
        }
      }
    });

    socket.on('makeMap', () => {
      webModelFunctions.update('makeMap', true);
    });

    socket.on('clearMap', () => {
      if (!webModel.ROSisRunning) {
        webModelFunctions.update('mapName', '');
      }
    });

    socket.on('gotoWayPoint', (data) => {
      wayPointEditor.goToWaypoint(data);
    });

    socket.on('returnToMapZeroPoint', () => {
      wayPointEditor.returnToMapZeroPoint();
    });

    // LocalMenu button handlers:
    socket.on('setWayPoint', (data) => {
      wayPointEditor.createWayPoint(data);
      setTimeout(wayPointEditor.updateWayPointList, 5000);
    });
    socket.on('tts', (data) => {
      tts(data);
    });
    socket.on('ask', (data) => {
      if (personalData.useMyCroft) {
        if (webModel.beQuiet) {
          webModelFunctions.update(
            'myCroftSaid',
            'I cannot reply, because I was asked to be quiet.',
          );
        } else {
          myCroft.injectText(data);
        }
      } else {
        tts(`I have no idea what you are talking about.`);
      }
    });
    socket.on('startROS', () => {
      if (webModel.logStreamerRunning) {
        stopLogStreamer();
      }
      webModelFunctions.update('ROSstart', true);
    });
    socket.on('stopROS', () => {
      webModelFunctions.update('ROSstart', false);
    });

    socket.on('haltRobot', () => {
      handleSemaphoreFiles.setSemaphoreFiles('stop');
    });
    socket.on('unHaltRobot', () => {
      handleSemaphoreFiles.setSemaphoreFiles('go');
    });
    socket.on('beQuiet', () => {
      handleSemaphoreFiles.setSemaphoreFiles('beQuiet');
    });
    socket.on('talk', () => {
      handleSemaphoreFiles.setSemaphoreFiles('talk');
    });

    socket.on('stopIdleTimer', () => {
      webModelFunctions.scrollingStatusUpdate('Idle Timer Stopped.');
      webModelFunctions.update('idleTimeout', false);
    });
    socket.on('startIdleTimer', () => {
      webModelFunctions.scrollingStatusUpdate('Idle Timer Restarted.');
      webModelFunctions.update('idleTimeout', true);
    });

    // unplugYourself
    socket.on('unplugYourself', () => {
      // Setting the status means if we flip the switch before ROS
      // is up, it should unplug once it is ready,
      // but ONLY when the behavior tree is ready.
      // TODO: Should unplugging be in the poll too?
      webModelFunctions.update('unplugYourself', true);
      // By calling the rosInterface here ,this allows us to unplug the robot
      // any time ROS is running, even if the behavior tree isn't at a point
      // where it wants to do this.
      rosInterface.unplugRobot(true);
    });
    socket.on('doNotUnplugYourself', () => {
      webModelFunctions.update('unplugYourself', false);
      // Again, this lets us override the behavior tree ASAP if we need to.
      rosInterface.unplugRobot(false);
    });

    // ROS Parameters
    socket.on('monitorAC', () => {
      rosInterface.setParam('monitorACconnection', true);
    });
    socket.on('ignoreAC', () => {
      rosInterface.setParam('monitorACconnection', false);
    });

    socket.on('monitorIR', () => {
      rosInterface.setParam('ignoreIRSensors', false);
    });
    socket.on('ignoreIR', () => {
      rosInterface.setParam('ignoreIRSensors', true);
    });

    socket.on('monitorCliff', () => {
      rosInterface.setParam('ignoreCliffSensors', false);
    });
    socket.on('ignoreCliff', () => {
      rosInterface.setParam('ignoreCliffSensors', true);
    });

    socket.on('monitorFloor', () => {
      rosInterface.setParam('ignoreFloorSensors', false);
    });
    socket.on('ignoreFloor', () => {
      rosInterface.setParam('ignoreFloorSensors', true);
    });

    socket.on('monitorProximity', () => {
      rosInterface.setParam('ignoreProximity', false);
    });
    socket.on('ignoreProximity', () => {
      rosInterface.setParam('ignoreProximity', true);
    });

    socket.on('saveMap', async (data) => {
      console.log(`Save map as: ${data}`);
      // TODO: Map names with spaces do not seem to work,
      //       Either fix that, or update the web interface to reject them.
      await saveMap(data);
    });
    socket.on('startLogStreamer', () => {
      startLogStreamer();
    });
    socket.on('stopLogStreamer', () => {
      stopLogStreamer();
    });
    socket.on('toggleLogStreamer', () => {
      if (webModel.logStreamerRunning) {
        stopLogStreamer();
      } else {
        startLogStreamer();
      }
    });
    socket.on('toggleDebug', () => {
      webModelFunctions.toggle('debugging');
    });
    socket.on('toggleLogConsoleMessages', () => {
      webModelFunctions.toggle('logConsoleMessages');
    });
    socket.on('toggleLogBehaviorMessages', () => {
      webModelFunctions.toggle('logBehaviorMessages');
    });
    socket.on('toggleLogTalkAboutEvents', () => {
      webModelFunctions.toggle('logTalkAboutEvents');
    });
    socket.on('toggleLogOtherMessages', () => {
      webModelFunctions.toggle('logOtherMessages');
    });
    socket.on('toggleCamera', () => {
      camera.toggle();
    });
    socket.on('toggleMasterRelay', () => {
      masterRelay('toggle');
    });
    socket.on('toggleRelay', (data) => {
      robotModel.usbRelay.toggle(data);
    });
    socket.on('toggleRelayByName', (data) => {
      robotModel.usbRelay.toggle(
        webModel.relays.find((x) => x.name === data).number,
      );
    });
    socket.on('exit', () => {
      console.log('Shutdown requested from web interface!');
      webModelFunctions.scrollingStatusUpdate(
        'Shutdown requested from web interface!',
      );
      webModelFunctions.update('shutdownRequested', true);
    });
    socket.on('arduino', () => {
      if (!personalData.demoWebSite) {
        if (webModel.neoPixelsOn) {
          arduino.lightsOut();
        } else {
          arduino.init();
        }
      } else {
        webModelFunctions.update('neoPixelsOn', !webModel.neoPixelsOn);
      }
    });
    socket.on('scrollingStatusUpdate', (data) => {
      webModelFunctions.scrollingStatusUpdate(data);
    });
    socket.on('getPersonalData', () => {
      socket.emit('personalData', personalData);
    });
    // socket.on('toggleMycroft', function () {
    //     myCroft.start();
    // });
  });
}

exports.start = start;
