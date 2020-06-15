const express = require('express');
const redis = require('redis');
const session = require('express-session');
const RedisStore = require('connect-redis')(session);
// Because Express.js says not to use their session store for production.
const jwt = require('jsonwebtoken'); // used to create, sign, and verify tokens
const cookieParser = require('cookie-parser');
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

const client = redis.createClient({
  host: 'localhost',
  // The default TTL is 24 hours
  // Plus the cookies themselves will die when the browser is closed.
  prefix: 'website-sessions:', // The ':' makes it a "folder" in redis
});
client.unref();
client.on('error', console.error);

const app = express();
app.use(cookieParser());
const hour = 3600000;
/** @namespace personalData.webSiteSettings */
// noinspection JSCheckFunctionSignatures
/** @namespace personalData.webSiteSettings.sessionSecret */
app.use(
  session({
    store: new RedisStore({ client }),
    cookie: { maxAge: hour },
    rolling: true,
    secret: personalData.webSiteSettings.sessionSecret,
    saveUninitialized: false, // True for built in, false for redis-connect
    resave: false, // True for built in, false for redis-connect
  }),
);

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

// Allow posting to root with a username and password for authentication.
app.post('/', (req, res) => {
  // TODO: I think this code is dead/not used, but I need to test to make sure before removing it.
  // Allow for local plaintext password (in case we are offline) by creating and sending ourselves a token.
  // This is kind of overkill, but I did it to test the system locally before building it remotely.
  /** @namespace personalData.webSiteSettings.basicAuthPassword */
  if (
    req.body.name &&
    req.body.password === personalData.webSiteSettings.basicAuthPassword
  ) {
    /* Token help:
     https://stormpath.com/blog/nodejs-jwt-create-verify
     https://scotch.io/tutorials/authenticate-a-node-js-api-with-json-web-tokens
     */
    /** @namespace personalData.webSiteSettings.tokenSecret */
    const token = jwt.sign(
      { name: req.body.name },
      personalData.webSiteSettings.tokenSecret,
      {
        expiresIn: '24h',
      },
    );
    console.log('Token:', token);
    // Not setting a cookie actually.
    // This would be used if I was using full token auth instead of only using the token as a means
    // to pass authentication from a remote server to myself here.
    // res.cookie('access_token', token);
    // Create a page to post the token back to ourselves:
    const postPage = `<!DOCTYPE html>
            <html lang="en">
            <head>
            <meta charset="UTF-8">
            <title>PostBack</title>
        </head>
        <body>
        <form name="postToken" action="/" method="post">
            <input type="hidden" name="token" value="${token}">
        </form>
        </body>
        <script type="text/javascript">
            window.onload = function() {
                console.log('.');
                document.forms["postToken"].submit();
            }
        </script>
        </html>`;
    res.set('Content-type', 'text/html');
    res.send(new Buffer(postPage));
  } else if (req.body.token) {
    // Use a token sent to us by a remote site or ourselves:
    console.log(req.body.token);
    // Set a junk string for the tokenSecret if you want to test how it operates on failure.
    jwt.verify(
      req.body.token,
      personalData.webSiteSettings.tokenSecret,
      (err, decoded) => {
        if (err) {
          res.redirect('/basicLogin.html');
        } else {
          console.log(decoded);
          console.log(`Creating new session for ${decoded.name}`);
          req.session.authorized = true;
          req.session.userName = decoded.name;
          res.redirect('/');
        }
      },
    );
  }
});

// Require session for all pages unless personalData.webSiteSettings.requirePassword is OFF:
app.use((req, res, next) => {
  /** @namespace personalData.webSiteSettings.requirePassword */
  if (
    req.url === '/basicLogin.html' ||
    !personalData.webSiteSettings.requirePassword
  ) {
    next();
  } else if (req.session.userName && req.session.authorized === true) {
    next();
  } else {
    res.redirect('/basicLogin.html');
  }
});

// All web content is housed in the website folder
app.use(express.static(`${__dirname}/../website/build`));

const handleSemaphoreFiles = require('./handleSemaphoreFiles');

const saveMap = function (newMapName) {
  // TODO: Positive feedback that map is saved.
  // TODO: If the map exists, maybe warn?
  const mapDir = `${process.env.HOME}/.arlobot/rosmaps/`;
  const serverMapProcess = new LaunchScript({
    name: 'SaveMap',
    callback: updateMapList,
    ROScommand: `rosrun map_server map_saver -f ${mapDir}${newMapName}`,
    scriptArguments: newMapName,
  });
  // console.log(serverMapProcess.ROScommand);
  serverMapProcess.start();
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
  const process = spawn(command, commandArgs);
  process.on('exit', () => {
    webModelFunctions.scrollingStatusUpdate('Log streamer killed');
    webModelFunctions.update('logStreamerRunning', false);
  });
  return process;
};

// This is a good example of integrating a simple ROS function into the web menu,
// without having to redesign everything.
// Place the buttons for things like this into the "Behavior" section,
// and only show them when ROS is started.
// Behaviors like these could also fall into "random activities" when robot is "idle",
// but then I think that it would need to be in the behavior tree
const startColorFollower = function () {
  webModelFunctions.scrollingStatusUpdate('Starting Color Follower.');
  const command = `${__dirname}/../scripts/object_follower.sh`;
  const colorFollowerProcess = spawn(command);
  colorFollowerProcess.stdout.setEncoding('utf8');
  colorFollowerProcess.stdout.on('data', (data) => {
    if (data.indexOf('ROI messages detected. Starting follower...') > -1) {
      webModelFunctions.update('colorFollowerRunning', true);
      webModelFunctions.scrollingStatusUpdate('Color Follower has started.');
      webModelFunctions.behaviorStatusUpdate('Color Follower started.');
    }
    // console.log(data);
  });
  colorFollowerProcess.stderr.setEncoding('utf8');
  colorFollowerProcess.stderr.on('data', (data) => {
    console.log('colorFollower:', data);
  });
  colorFollowerProcess.on('error', (err) => {
    console.error(err);
  });
  colorFollowerProcess.on('exit', (code) => {
    // Will catch multiple exit codes I think:
    if (code === 0) {
      webModelFunctions.scrollingStatusUpdate('Color Follower ended normally.');
    } else {
      console.log(`Color Follower failed with code: ${code}`);
    }
    webModelFunctions.update('colorFollowerRunning', false);
  });
  return colorFollowerProcess;
};
const stopColorFollower = function () {
  // pkill -f "roslaunch arlobot_launchers object_follower.launch"
  const command = '/usr/bin/pkill';
  const commandArgs = [
    '-f',
    'roslaunch arlobot_launchers object_follower.launch',
  ];
  const process = spawn(command, commandArgs);
  return process;
};

async function start() {
  /** @namespace personalData.webServerPort */
  const webServer = app.listen(personalData.webServerPort);
  const io = socketIo.listen(webServer);

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
        if (data === 'Explore!') {
          webModelFunctions.update('mapName', '');
          webModelFunctions.update('autoExplore', true);
        } else if (webModel.mapList.indexOf(data)) {
          webModelFunctions.update('autoExplore', false);
          webModelFunctions.update('mapName', data);
          wayPointEditor.updateWayPointList();
        }
      }
    });

    socket.on('makeMapGmapping', () => {
      webModelFunctions.update('makeMapGmapping', true);
    });

    socket.on('makeMapCartographer', () => {
      webModelFunctions.update('makeMapCartographer', true);
    });

    socket.on('clearMap', () => {
      if (!webModel.ROSisRunning) {
        webModelFunctions.update('autoExplore', false);
        webModelFunctions.update('mapName', '');
      }
    });

    socket.on('gotoWayPoint', (data) => {
      if (data) {
        if (webModel.wayPoints.indexOf(data) > -1) {
          webModelFunctions.updateWayPointNavigator('wayPointName', data);
          wayPointEditor.getWayPoint(data, (response) => {
            robotModel.wayPointNavigator.destinaitonWaypoint = response;
            webModelFunctions.updateWayPointNavigator('goToWaypoint', true);
          });
        }
      }
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
    socket.on('markDoorsClosed', () => {
      handleSemaphoreFiles.setSemaphoreFiles('markDoorsClosed');
    });

    socket.on('stopIdleTimer', () => {
      webModelFunctions.scrollingStatusUpdate('Idle Timer Stopped.');
      webModelFunctions.update('idleTimeout', false);
    });
    socket.on('startIdleTimer', () => {
      webModelFunctions.scrollingStatusUpdate('Idle Timer Restarted.');
      webModelFunctions.update('idleTimeout', true);
    });

    socket.on('pauseAutoExplore', () => {
      webModelFunctions.update('pauseExplore', true);
    });
    socket.on('unPauseAutoExplore', () => {
      webModelFunctions.update('pauseExplore', false);
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

    socket.on('saveMap', (data) => {
      console.log(`Save map as: ${data}`);
      saveMap(data);
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
    socket.on('startColorFollower', () => {
      if (!personalData.demoWebSite) {
        startColorFollower();
      } else {
        webModelFunctions.scrollingStatusUpdate('Color Follower has started.');
        webModelFunctions.behaviorStatusUpdate('Color Follower started.');
        webModelFunctions.update('colorFollowerRunning', true);
      }
    });
    socket.on('stopColorFollower', () => {
      if (!personalData.demoWebSite) {
        stopColorFollower();
      } else {
        webModelFunctions.scrollingStatusUpdate('Color Follower has stopped.');
        webModelFunctions.behaviorStatusUpdate('Color Follower stopped.');
        webModelFunctions.update('colorFollowerRunning', false);
      }
    });
    socket.on('toggleDebug', () => {
      webModelFunctions.toggle('debugging');
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
