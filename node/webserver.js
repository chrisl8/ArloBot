const personalData = require('./personalData');
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
const Camera = require('./Camera');
/** @namespace personalData.camera0name */
const camera = new Camera('Camera', personalData.camera0name);
const Arduino = require('./Arduino');
const arduino = new Arduino(true);
const robotModel = require('./robotModel');
const LaunchScript = require('./LaunchScript');
const tts = require('./tts');
const myCroft = require('./MyCroft');

const WayPoints = require('./WayPoints.js');
const wayPointEditor = new WayPoints();

const rosInterface = require('./rosInterface');

const express = require('express');
const session = require('express-session');
const RedisStore = require('connect-redis')(session);
// Because Express.js says not to use their session store for production.
const jwt = require('jsonwebtoken'); // used to create, sign, and verify tokens
const cookieParser = require('cookie-parser');

const spawn = require('child_process').spawn;
const bodyParser = require('body-parser');
const masterRelay = require('./MasterRelay');

const updateMapList = require('./updateMapList');
updateMapList();

const app = express();
app.use(cookieParser());
const hour = 3600000;
/** @namespace personalData.webSiteSettings */
//noinspection JSCheckFunctionSignatures
/** @namespace personalData.webSiteSettings.sessionSecret */
app.use(session({
    store: new RedisStore({
        host: 'localhost',
        // The default TTL is 24 hours
        // Plus the cookies themselves will die when the browser is closed.
        prefix: 'website-sessions:' // The ':' makes it a "folder" in redis
    }),
    cookie: {maxAge: hour},
    rolling: true,
    secret: personalData.webSiteSettings.sessionSecret,
    saveUninitialized: false, // True for built in, false for redis-connect
    resave: false // True for built in, false for redis-connect
}));

// For json encoded post requests, which I use:
app.use(bodyParser.json());
// Required for Twilio:
app.use(bodyParser.urlencoded({
    extended: true
}));
// var path = require('path');
// app.get('/', isLoggedIn, function (req, res) {
//     console.log('get /');
//     res.sendFile(path.join(__dirname + '/../website/index.html'));
// });

// Allow posting to root with a username and password for authentication.
app.post('/', function (req, res) {
    // Allow for local plaintext password (in case we are offline) by creating and sending ourselves a token.
    // This is kind of overkill, but I did it to test the system locally before building it remotely.
    /** @namespace personalData.webSiteSettings.basicAuthPassword */
    if (req.body.name && req.body.password === personalData.webSiteSettings.basicAuthPassword) {
        /* Token help:
         https://stormpath.com/blog/nodejs-jwt-create-verify
         https://scotch.io/tutorials/authenticate-a-node-js-api-with-json-web-tokens
         */
        /** @namespace personalData.webSiteSettings.tokenSecret */
        const token = jwt.sign({name: req.body.name}, personalData.webSiteSettings.tokenSecret, {
            expiresIn: '24h'
        });
        console.log('Token:', token);
        // Not setting a cookie actually.
        // This would be use dif I was using full token auth instaed of only using the token as a means
        // to pass authentication from a remote server to myself here.
        // res.cookie('access_token', token);
        // Create a page to post the token back to ourselves:
        let postPage = `<!DOCTYPE html>
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
        jwt.verify(req.body.token, personalData.webSiteSettings.tokenSecret, function (err, decoded) {
            if (err) {
                res.redirect('/basicLogin.html');
            } else {
                console.log(decoded);
                console.log(`Creating new session for ${decoded.name}`);
                req.session.authorized = true;
                req.session.userName = decoded.name;
                res.redirect('/');
            }
        });
    }
});

// Require session for all pages unless personalData.webSiteSettings.requirePassword is OFF:
app.use(function (req, res, next) {
    /** @namespace personalData.webSiteSettings.requirePassword */
    if (req.url === '/basicLogin.html' || !personalData.webSiteSettings.requirePassword) {
        next();
    } else {
        // if (req.cookies.access_token) { // Use a token.
        //     console.log("Acces Token :  ", req.cookies.access_token);
        //     // Set a junk string for the tokenSecret if you want to test how it operates on failure.
        //     jwt.verify(req.cookies.access_token, personalData.webSiteSettings.tokenSecret, function (err, decoded) {
        //         if (err) {
        //             res.redirect('/basicLogin.html');
        //         } else {
        //             console.log(decoded);
        //             next();
        //         }
        //     });
        // } else { // Or else a session?
        if (req.session.userName && req.session.authorized === true) {
            next();
        } else {
            res.redirect('/basicLogin.html');
        }
        // }
    }
});

// All web content is housed in the website folder
app.use(express.static(__dirname + '/../website/build'));

const handleSemaphoreFiles = require('./handleSemaphoreFiles');
handleSemaphoreFiles.readSemaphoreFiles();

const saveMap = function (newMapName) {
    // TODO: Positive feedback that map is saved.
    // TODO: If the map exists, maybe warn?
    const mapDir = process.env.HOME + '/.arlobot/rosmaps/';
    let serverMapProcess = new LaunchScript({
        name: 'SaveMap',
        callback: updateMapList,
        ROScommand: 'rosrun map_server map_saver -f ' + mapDir + newMapName,
        scriptArguments: newMapName
    });
    //console.log(serverMapProcess.ROScommand);
    serverMapProcess.start();
};

const startLogStreamer = function () {
    const command = __dirname + '/../scripts/log-watcher.sh';
    const logStreamerProcess = spawn(command);
    logStreamerProcess.stdout.setEncoding('utf8');
    logStreamerProcess.stdout.on('data', function (data) {
        //console.log(data);
    });
    logStreamerProcess.stderr.setEncoding('utf8');
    logStreamerProcess.stderr.on('data', function (data) {
        //console.log(data);
    });
    logStreamerProcess.on('error', function (err) {
        //console.log(err);
    });
    logStreamerProcess.on('exit', function (code) {
        // Will catch multiple exit codes I think:
        if (code === 0) {
            webModelFunctions.scrollingStatusUpdate('Log streamer started');
            webModelFunctions.update('logStreamerRunning', true);
        } else {
            console.log('Log streamer failed with code: ' + code);
            webModelFunctions.update('logStreamerRunning', false);
        }
    });
    return logStreamerProcess;
};
const stopLogStreamer = function () {
    const command = '/usr/bin/pkill';
    const commandArgs = ['-f', 'log.io'];
    const process = spawn(command, commandArgs);
    process.stdout.setEncoding('utf8');
    process.stdout.on('data', function (data) {
        //console.log(data);
    });
    process.stderr.setEncoding('utf8');
    process.stderr.on('data', function (data) {
        //console.log(data);
    });
    process.on('error', function (err) {
        //console.log(err);
    });
    process.on('exit', function () { // Argument Options: code
        //console.log(code);
        webModelFunctions.scrollingStatusUpdate('Log streamer killed');
        webModelFunctions.update('logStreamerRunning', false);
    });
    return process;
};

// TODO: Shouldn't this be a behavior in along with Explore and Load Map?
// This is a good example of integrating a simple ROS function into the web menu,
// without having to redesign everything.
// Place the buttons for things like this into the "Behavior" section,
// and only show them when ROS is starting.
// Behaviors like these could also fall into "ramdon activities" when robot is "idle",
// but then I think that it would need to be in the behavior tree
const startColorFollower = function () {
    webModelFunctions.scrollingStatusUpdate('Starting Color Follower.');
    const command = __dirname + '/../scripts/object_follower.sh';
    const colorFollowerProcess = spawn(command);
    colorFollowerProcess.stdout.setEncoding('utf8');
    colorFollowerProcess.stdout.on('data', function (data) {
        if (data.indexOf('ROI messages detected. Starting follower...') > -1) {
            webModelFunctions.update('colorFollowerRunning', true);
            webModelFunctions.scrollingStatusUpdate('Color Follower has started.');
            webModelFunctions.behaviorStatusUpdate('Color Follower started.');
        }
        // console.log(data);
    });
    colorFollowerProcess.stderr.setEncoding('utf8');
    colorFollowerProcess.stderr.on('data', function (data) {
        console.log('colorFollower:', data);
    });
    colorFollowerProcess.on('error', function (err) {
        //console.log(err);
    });
    colorFollowerProcess.on('exit', function (code) {
        // Will catch multiple exit codes I think:
        if (code === 0) {
            webModelFunctions.scrollingStatusUpdate('Color Follower ended normally.');
        } else {
            console.log('Color Follower failed with code: ' + code);
        }
        webModelFunctions.update('colorFollowerRunning', false);
    });
    return colorFollowerProcess;
};
const stopColorFollower = function () {
    //pkill -f "roslaunch arlobot_launchers object_follower.launch"
    const command = '/usr/bin/pkill';
    const commandArgs = ['-f', 'roslaunch arlobot_launchers object_follower.launch'];
    const process = spawn(command, commandArgs);
    process.stdout.setEncoding('utf8');
    process.stdout.on('data', function (data) {
        //console.log(data);
    });
    process.stderr.setEncoding('utf8');
    process.stderr.on('data', function (data) {
        //console.log(data);
    });
    process.on('error', function (err) {
        //console.log(err);
    });
    process.on('exit', function (code) {
        //console.log(code);
    });
    return process;
};

async function start() {
    /** @namespace personalData.webServerPort */
    const webServer = app.listen(personalData.webServerPort);
    const io = require("socket.io").listen(webServer);

    webModelFunctions.emitter.on('change', function () {
        io.sockets.emit('webModel', webModel);
    });

    // Socket listeners
    io.on('connection', function (socket) {
        socket.emit('startup', webModel);
        const address = socket.request.connection.remoteAddress;
        console.log("Web connection from " + address);

        socket.on('setMap', function (data) {
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

        socket.on('makeMap', () => {
           webModelFunctions.update('makeMap', true);
        });

        socket.on('clearMap', function () {
            if (!webModel.ROSisRunning) {
                webModelFunctions.update('autoExplore', false);
                webModelFunctions.update('mapName', '');
            }
        });

        socket.on('gotoWayPoint', function (data) {
            if (data) {
                if (webModel.wayPoints.indexOf(data) > -1) {
                    webModelFunctions.updateWayPointNavigator('wayPointName', data);
                    wayPointEditor.getWayPoint(data, function (response) {
                        robotModel.wayPointNavigator.destinaitonWaypoint = response;
                        webModelFunctions.updateWayPointNavigator('goToWaypoint', true);
                    });
                }
            }
        });

        // LocalMenu button handlers:
        socket.on('setWayPoint', function (data) {
            wayPointEditor.createWayPoint(data);
            setTimeout(wayPointEditor.updateWayPointList, 5000);
        });
        socket.on('tts', function (data) {
            tts(data);
        });
        socket.on('ask', function (data) {
            if (personalData.useMyCroft) {
              if (webModel.beQuiet) {
                webModelFunctions.update('myCroftSaid', 'I cannot reply, because I was asked to be quiet.');
              } else {
                myCroft.injectText(data);
              }
            } else {
                tts(`I have no idea what you are talking about.`);
            }
        });
        socket.on('startROS', function () {
            if (webModel.logStreamerRunning) {
                stopLogStreamer();
            }
            webModelFunctions.update('ROSstart', true);
        });
        socket.on('stopROS', function () {
            webModelFunctions.update('ROSstart', false);
        });

        socket.on('haltRobot', function () {
            handleSemaphoreFiles.setSemaphoreFiles('stop');
        });
        socket.on('unHaltRobot', function () {
            handleSemaphoreFiles.setSemaphoreFiles('go');
        });
        socket.on('beQuiet', function () {
            handleSemaphoreFiles.setSemaphoreFiles('beQuiet');
        });
        socket.on('talk', function () {
            handleSemaphoreFiles.setSemaphoreFiles('talk');
        });
        socket.on('markDoorsClosed', function () {
            handleSemaphoreFiles.setSemaphoreFiles('markDoorsClosed');
        });

        socket.on('stopIdleTimer', function () {
            webModelFunctions.scrollingStatusUpdate("Idle Timer Stopped.");
            webModelFunctions.update('idleTimeout', false);
        });
        socket.on('startIdleTimer', function () {
            webModelFunctions.scrollingStatusUpdate("Idle Timer Restarted.");
            webModelFunctions.update('idleTimeout', true);
        });

        socket.on('pauseAutoExplore', function () {
            webModelFunctions.update('pauseExplore', true);
        });
        socket.on('unPauseAutoExplore', function () {
            webModelFunctions.update('pauseExplore', false);
        });
        // unplugYourself
        socket.on('unplugYourself', function () {
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
        socket.on('doNotUnplugYourself', function () {
            webModelFunctions.update('unplugYourself', false);
            // Again, this lets us override the behavior tree ASAP if we need to.
            rosInterface.unplugRobot(false);
        });

        // ROS Parameters
        socket.on('monitorAC', function () {
            rosInterface.setParam('monitorACconnection', true);
        });
        socket.on('ignoreAC', function () {
            rosInterface.setParam('monitorACconnection', false);
        });

        socket.on('monitorIR', function () {
            rosInterface.setParam('ignoreIRSensors', false);
        });
        socket.on('ignoreIR', function () {
            rosInterface.setParam('ignoreIRSensors', true);
        });

        socket.on('monitorCliff', function () {
            rosInterface.setParam('ignoreCliffSensors', false);
        });
        socket.on('ignoreCliff', function () {
            rosInterface.setParam('ignoreCliffSensors', true);
        });

        socket.on('monitorFloor', function () {
            rosInterface.setParam('ignoreFloorSensors', false);
        });
        socket.on('ignoreFloor', function () {
            rosInterface.setParam('ignoreFloorSensors', true);
        });

        socket.on('monitorProximity', function () {
            rosInterface.setParam('ignoreProximity', false);
        });
        socket.on('ignoreProximity', function () {
            rosInterface.setParam('ignoreProximity', true);
        });

        socket.on('saveMap', function (data) {
            console.log('Save map as: ' + data);
            saveMap(data);
        });
        socket.on('startLogStreamer', function () {
            startLogStreamer();
        });
        socket.on('stopLogStreamer', function () {
            stopLogStreamer();
        });
        socket.on('toggleLogStreamer', function () {
            if (webModel.logStreamerRunning) {
                stopLogStreamer();
            } else {
                startLogStreamer();
            }
        });
        socket.on('startColorFollower', function () {
            if (!personalData.demoWebSite) {
                startColorFollower();
            } else {
                webModelFunctions.scrollingStatusUpdate('Color Follower has started.');
                webModelFunctions.behaviorStatusUpdate('Color Follower started.');
                webModelFunctions.update('colorFollowerRunning', true);
            }
        });
        socket.on('stopColorFollower', function () {
            if (!personalData.demoWebSite) {
                stopColorFollower();
            } else {
                webModelFunctions.scrollingStatusUpdate('Color Follower has stopped.');
                webModelFunctions.behaviorStatusUpdate('Color Follower stopped.');
                webModelFunctions.update('colorFollowerRunning', false);
            }
        });
        socket.on('toggleDebug', function () {
            webModelFunctions.toggle('debugging');
        });
        socket.on('toggleCamera', function () {
            camera.toggle();
        });
        socket.on('toggleMasterRelay', function () {
            masterRelay('toggle');
        });
        socket.on('toggleRelay', function (data) {
            robotModel.usbRelay.toggle(data);
        });
        socket.on('toggleRelayByName', function (data) {
            robotModel.usbRelay.toggle(webModel.relays.find(x => x.name === data)['number']);
        });
        socket.on('exit', function () {
            console.log('Shutdown requested from web interface!');
            webModelFunctions.scrollingStatusUpdate('Shutdown requested from web interface!');
            webModelFunctions.update('shutdownRequested', true);
        });
        socket.on('arduino', function () {
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
        socket.on('scrollingStatusUpdate', data => {
            webModelFunctions.scrollingStatusUpdate(data);
        });
        // socket.on('toggleMycroft', function () {
        //     myCroft.start();
        // });
    });
}

exports.start = start;
