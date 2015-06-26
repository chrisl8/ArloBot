// http://behavior3js.guineashots.com/
// Setup:
// npm install behavior3js
// Modifications:
// node_modules/behavior3js/package.json:
//  -"main": "libs/b3core.0.1.0.min.js",
//  +"main": "libs/b3core.0.1.0.js",
// node_modules/behavior3js/libs/b3core.0.1.0.js:
//   this.b3 = this.b3 || {};
//  +b3 = {};

var webModel = require('./webModel');

var fs = require('fs');

// Load personal settings not included in git repo
var personalDataFile = process.env.HOME + '/.arlobot/personalDataForBehavior.json';
var personalData = JSON.parse(fs.readFileSync(personalDataFile, 'utf8'));

b3 = {};
var behavior3js = require('behavior3js');

var exec = require('child_process').exec;
var spawn = require('child_process').spawn;
// Note that tts will convert text to speech,
// or it will send a ".wav" string (path, etc)
// to aplay.
// The benefit of using it is that it will honor
// system wide "bequiet" requests.
// The same modele is used by ROS Python scripts.
var tts = require('./tts');
var LaunchScript = require('./launch_script');

var textme = require('./textme');
var arloTree = new b3.BehaviorTree();

// Cleanup on shutdown
// http://stackoverflow.com/questions/14031763/doing-a-cleanup-action-just-before-node-js-exits
var kill_rosHasRun = false;
var killROS = function(exitWhenDone) {
    var command = __dirname + '/../scripts/kill_ros.sh';
    // It is rather catastrophic if this repeats!
    if (!kill_rosHasRun) {
        kill_rosHasRun = true;
        webModel.ROSstart = false;
        console.log("Running kill_ros.sh . . .");
        var shutdownCommand = exec(command);
        shutdownCommand.stdout.on('data', function(data) {
            console.log('kill_ros: ' + data);
        });

        shutdownCommand.stderr.on('data', function(data) {
            console.log('kill_ros: ' + data);
        });

        shutdownCommand.on('close', function(code) {
            console.log('kill_ros.sh closed with code ' + code);
            if (exitWhenDone) {
                process.exit();
            } else {
                kill_rosHasRun = false;
                webModel.ROSisRunning = false;
            }
        });
        shutdownCommand.on('error', function(err) {
            console.log('kill_ros process error' + err);
        });
    }
};

function exitHandler(options, err) {
    if (options.cleanup) console.log('arloBehavior has been told to "cleanup", doing nothing.');
    if (err) console.log(err.stack);
    //if (options.exit) process.exit();
    if (options.exit) {
        console.log('arloBehavior has been asked to exit, calling killROS');
        killROS(true);
    }
}

//do something when app is closing
process.on('exit', exitHandler.bind(null, {
    cleanup: true
}));

//catches ctrl+c event
process.on('SIGINT', exitHandler.bind(null, {
    exit: true
}));

//catches uncaught exceptions
process.on('uncaughtException', exitHandler.bind(null, {
    exit: true
}));

// Start Webserver
/*
// This is a good example of using forever, but we do not need it here.
var webserver = new(forever.Monitor)('webserver.js', {
    //max: 3,
    silent: false,
    sourceDir: 'webserver/',
    args: [],
});
webserver.on('exit', function() {
    console.log('Webserver has exited after 10 restarts');
});
console.log("Starting web server.");
webserver.start();
*/
var webServer = require('./webserver');
var io = webServer.start();

// TODO: REMOVE this.
var ROSrunRequested = b3.Class(b3.Condition);
ROSrunRequested.prototype.name = 'ROSrunRequested';
ROSrunRequested.prototype.tick = function(tick) {
    return b3.SUCCESS;
};

// TODO: Perfect this pattern and replicate to all script starting behaviors.
var StartROS = b3.Class(b3.Action);
StartROS.prototype.name = 'StartROS';
StartROS.prototype.tick = function(tick) {
    // ROS Process launch behavior pattern:
    // FIRST: This decides if we run this process or not:
    if (!webModel.ROSstart) {
        console.log(this.name + ': FAILURE');
        return b3.FAILURE;
    } else {
        if (arloBot.ROSprocess.started) {
            // startupComplete indicates either:
                // Script exited
                // Script threw "success string"
                // Script returned any data if it wasn't given a "success string"
            if (arloBot.ROSprocess.startupComplete) {
                if (arloBot.ROSprocess.hasExited) {
                    // Once the process has exited:
                    // 1. DISABLE whatever user action causes it to be called,
                    // so that it won't loop.
                    webModel.ROSstart = false;
                    // 2. Now that it won't loop, set .started to false,
                    // so that it can be run again.
                    arloBot.ROSprocess.started = false;
                    // 3. Send a status to the web site:
                    webModel.status = 'ROS process has closed.';
                    // 4. Log the closure to the console,
                    // because this is significant.
                    console.log(this.name + "Process Closed.");
                    // 5. Set any special status flags for this
                    // process. i.e. ROSisRunning sets the start/stop button position
                    webModel.ROSisRunning = false;
                    // 6. Any special "cleanup" required?
                    // In this case we will run the kill routine.
                    killROS(false);
                    // Leave it 'RUNNING' and
                    // let the next Behavior tick respond as it would,
                    // if this function was never requested.
                    return b3.RUNNING;
                } else {
                    // This is where we go if the start is complete,
                    // and did not fail.
                    // This will repeat on every tick!
                    // 1. Set any special status flags for this
                    // process. i.e. ROSisRunning sets the start/stop button position
                    webModel.ROSisRunning = true;
                    // Whether we return 'RUNNING' or 'SUCCESS',
                    // is dependent on how this Behavior node works.
                    return b3.SUCCESS;
                }
            } else {
                console.log(this.name + " Starting up . . .");
                return b3.RUNNING;
            }
        } else {
            // IF the process is supposed to start, but wasn't,
            // then run it:
            arloBot.ROSprocess.start();
            console.log(this.name + " Process starting!");
            return b3.RUNNING;
        }
    }
};

var getMapOrExploreRequest = b3.Class(b3.Action);
getMapOrExploreRequest.prototype.name = 'getMapOrExploreRequest';
getMapOrExploreRequest.prototype.tick = function(tick) {
    if (webServer.webModel.autoExplore) {
        return b3.SUCCESS;
    }
    if (webServer.webModel.mapName !== '') {
        return b3.SUCCESS;
    }
    if (!arloBot.whereamiTextSent) {
        textme('Where am I?');
        arloBot.whereamiTextSent = true;
    }
    console.log(this.name);
    return b3.RUNNING;
};

var UnPlugRobot = b3.Class(b3.Action);
UnPlugRobot.prototype.name = 'UnPlugRobot';
UnPlugRobot.prototype.tick = function(tick) {
    if (!webModel.pluggedIn) return b3.SUCCESS;
    if (webModel.ignorePluggedIn) return b3.SUCCESS;
    if (webServer.webModel.laptopFullyCharged) {
        if (!arloBot.unplugMeTextSent) {
            textme('Please unplug me!');
            arloBot.unplugMeTextSent = true;
        }
    }
    console.log(this.name);
    return b3.FAILURE;
};

var AutoExplore = b3.Class(b3.Action);
AutoExplore.prototype.name = 'AutoExplore';
AutoExplore.prototype.tick = function(tick) {
    if (webServer.webModel.autoExplore) {
        if (arloBot.exploreProcess.started) {

            // Catch changes in pauseExplore and send them to the arlobot_explore pause_explorer service
            if (arloBot.pauseExplore !== webServer.webModel.pauseExplore) {
                // TODO: Should this use the LaunchScript object?
                arloBot.pauseExplore = webServer.webModel.pauseExplore;
                var command = '/opt/ros/indigo/bin/rosservice call /arlobot_explore/pause_explorer ' + arloBot.pauseExplore;
                exec(command);
            }

            if (arloBot.exploreProcess.startupComplete) {
                if (arloBot.exploreProcess.hasExited) {
                    webServer.webModel.autoExplore = false;
                    arloBot.exploreProcess.started = false;
                    console.log(this.name + "FAILURE");
                    return b3.FAILURE;
                } else {
                    webModel.status = 'Robot is Exploring!';
                    console.log('Robot is Exploring!');
                    return b3.RUNNING;
                }
            } else {
                webModel.status = 'Explore process is starting...';
                console.log('Explore process is starting...');
                return b3.RUNNING;
            }
        } else {
            arloBot.exploreProcess.start();
            console.log(this.name);
            return b3.RUNNING;
        }
    } else {
        // Return success if we were NOT asked to explore,
        // thus passing behavior tree on to LoadMap
        return b3.SUCCESS;
    }
};

// TODO: Turn the LaunchScript process running into a clear pattern.
var LoadMap = b3.Class(b3.Action);
LoadMap.prototype.name = 'LoadMap';
LoadMap.prototype.tick = function(tick) {
    // ROS Process launch behavior pattern:
    // FIRST: This decides if we run this process or not:
    if (webServer.webModel.mapName === '') {
        // This fails if we have no map name.
        console.log(this.name + ' FAILURE');
        return b3.FAILURE;
    } else {
        if (arloBot.loadMapProcess.started) {
            if (arloBot.loadMapProcess.startupComplete) {
                if (arloBot.loadMapProcess.hasExited) {
                    // Once the process has exited:
                    // 1. DISABLE whatever user action causes it to be called,
                    // so that it won't loop.
                    webServer.webModel.mapName = '';
                    // 2. Now that it won't loop, set .started to false,
                    // so that it can be run again.
                    arloBot.loadMapProcess.started = false;
                    // 3. Send a status to the web site:
                    webModel.status = 'Map process has closed.';
                    // 4. Log the closure to the console,
                    // because this is significant.
                    console.log(this.name + "Process Closed.");
                    // Leave it 'RUNNING' and
                    // let the next Behavior tick respond as it would,
                    // if this function was never requested.
                    return b3.RUNNING;
                } else {
                    // This will repeat on every tick!
                    webModel.status = 'Map is Loaded.';
                    // Whether we return 'RUNNING' or 'SUCCESS',
                    // is dependent on how this Behavior node works.
                    return b3.RUNNING;
                }
            } else {
                console.log(this.name + " Starting up . . .")
                return b3.RUNNING;
            }
        } else {
            // IF the process is supposed to start, but wasn't,
            // then run it:
            console.log('Map: ' + webServer.webModel.mapName);
            //arloBot.loadMapProcess.scriptArguments = [webServer.webModel.mapName];
            arloBot.loadMapProcess.ROScommand = arloBot.loadMapProcess.ROScommand + process.env.HOME + '/.arlobot/rosmaps/' + webModel.mapName + '.yaml';
            console.log(arloBot.loadMapProcess.ROScommand);
            arloBot.loadMapProcess.start();
            return b3.RUNNING;
        }
    }
};

var MyAction2 = b3.Class(b3.Action);
MyAction2.prototype.name = 'MyAction2';
MyAction2.prototype.tick = function(tick) {
    console.log(this.name);
    return b3.FAILURE;
};

// Build this file with http://behavior3js.guineashots.com/editor/#
// and you can LOAD this data into the editor to start where you left off again
var arloNodeData = JSON.parse(fs.readFileSync('arloTreeData.json', 'utf8'));

// Despite the Editor creating a beautiful JSON behavior tree for us,
// we still have to list the custom nodes for it by hand.
//var customNodeNames = {
//'ROSisRunning': ROSisRunning,
//'StartROS': StartROS,
//'UnPlugRobot': UnPlugRobot,
//'LaptopBatteryCharged': LaptopBatteryCharged,
//'RobotIsUnplugged': RobotIsUnplugged
//};
var customNodeNames = {};

function parseCustomNodes(element, index, array) {
    customNodeNames[element.name] = eval(element.name);
}
arloNodeData.custom_nodes.forEach(parseCustomNodes);

arloTree.load(arloNodeData, customNodeNames);

// This is the Arlobot "object" we will use to track everything,
// note that there is also a "blackboard" where nodes can track
// their internal status, even amongst multiple calls
// to the same node type,
// but this will keep track of broader robot settings.
// NOTE: Do NOT track things here that the nodes
// should be collecting from real time status!
// Rather track things we can only know from inside of here,
// such as last action or speech time, etc.
var arloBot = {
    whereamiTextSent: false,
    unplugMeTextSent: false,
    fullyCharged: false,

    pauseExplore: false
};

// TODO: Be sure to put 'unbuffer ' in front of any ROS commands
// if you hope to monitor the output,
// otherwise they never flush!
    // http://stackoverflow.com/a/11337310
    // http://linux.die.net/man/1/unbuffer

arloBot.ROSprocess = new LaunchScript({
    name: 'ROS',
    scriptName: '../scripts/start-metatron.sh',
    successString: 'process[arlobot-6]: started with pid'
});

var exploreCommand;
if (personalData.use_xv11) {
    exploreCommand = 'unbuffer roslaunch metatron_launchers add_autonomous_explore_xv11.launch';
} else {
    exploreCommand = 'unbuffer roslaunch metatron_launchers add_autonomous_explore.launch';
}
arloBot.exploreProcess = new LaunchScript({
    name: 'Explore',
    ROScommand: exploreCommand,
    successString: 'odom received'
});

var loadMapCommand;
if (personalData.use_xv11) {
    loadMapCommand = 'unbuffer roslaunch metatron_launchers load_map_xv11.launch map_file:=';
} else {
    loadMapCommand = 'unbuffer roslaunch metatron_launchers load_map.launch map_file:=';
}
arloBot.loadMapProcess = new LaunchScript({
    name: 'LoadMap',
    ROScommand: loadMapCommand,
    successString: 'odom received'
});

var blackboard = new b3.Blackboard();

console.log('start');
webServer.updateWebModel('status', 'Behavior Tree is running.');
setInterval(function() {
    arloTree.tick(arloBot, blackboard);
    // Note that we can do stuff between ticks if we want to,
    // although tracking information in the arloBot object,
    // or node data in the blackboard is preferable.
    if (arloBot.spokeThisTick) {
        arloBot.spokeLastTick = true;
    } else {
        arloBot.spokeLastTick = false;
    }
    arloBot.spokeThisTick = false;

    // Check laptop battery each tick
    // TODO: I have TWO batteries!
    var batteryCommand = '/usr/bin/upower -d|grep percentage|head -1';
    var batteryCheck = exec(batteryCommand);
    batteryCheck.stdout.on('data', function(data) {
        var re = /\s+/;
        webServer.updateWebModel('laptopBatteryPercentage', data.split(re)[2]);
        if (webServer.webModel.laptopBatteryPercentage.match('100%')) webServer.updateWebModel('laptopFullyCharged', true);
        else webServer.updateWebModel('laptopFullyCharged', false);
    });
    // Check plugged in status
    var powerCommand = '/usr/bin/upower -d|grep online';
    var powerCheck = exec(powerCommand);
    powerCheck.stdout.on('data', function(data) {
        if (data.match('no')) {
            webModel.pluggedIn = false;
        }
        if (data.match('yes')) {
            webModel.pluggedIn = true;
        }
    });

    //console.log(blackboard);

    if (webServer.webModel.shutdownRequested) killROS(true);

    console.log("---");
}, 1000);
//tree.tick(arloBot,blackboard);
// Copied from arloweb.js
var connectedToROS = false, // Track my opinion of the connection
    connectRequested = false, // For when we asked and are waiting patiently.
    pleaseWait = false, // Display Please Wait on the Connect button
    ros, // Empty global for actual connection.
    cmdVel, // Empty global for actual topic
    wakeScreen, // Empty global for actual topic
    toggleCamera, // Empty global for actual topic
    toggleRelay, // Empty global for actual topic
    sendTextToSpeak, // Empty global for actual topic
    shortDelay = 1000,
    longDelay = 3000,
    camera1On = false, // For tracking Camera status
    camera2On = false, // For tracking Camera status
    upperLightOn = false, // For tracking LED light bars
    lowerLightOn = false, // For tracking LED light bars
    UpperLightRowRelay = -1,
    UpperLightRowName = "TopLightRow", // Found in arlobot_usbrelay/param/usbrelay.yaml
    LowerLightRowRelay = -1,
    LowerLightRowName = "BottomLightRow", // Found in arlobot_usbrelay/param/usbrelay.yaml
    RightMotorRelay = -1,
    RightMotorName = "RightMotor", // Found in arlobot_usbrelay/param/usbrelay.yaml
    LeftMotorRelay = -1,
    LeftMotorName = "LeftMotor"; // Found in arlobot_usbrelay/param/usbrelay.yaml

/* TODO: This is all offline until this "bug" is fixed:
https://github.com/RobotWebTools/roslibjs/issues/160
var ROSLIB = require('roslib');
// Copied from arloweb.js
// Be sure to set url to point to localhost,
// and change any references to web objects with console.log (i.e. setActionField)

var talkToROS = function() {
    ros.getParams(function(params) {
        console.log(params);
    });
};

var pollROS = function() {
    console.log('ROSLIB pollROS run');
    connectedToROS = false;

    ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090',
        // TODO: This eliminates a warning about utf8, but does it work?
        encoding: 'ascii'
    });

    ros.on('connection', function() {
        console.log('Websocket connected.');
        //connectRequested = true;
        //updateConnectedButton();
        //checkROSServices();
        talkToROS();
    });

    ros.on('error', function(error) {
        //console.log('Error connecting to websocket server: ', error);
        console.log('ROSLIB Websocket error');
        if (ros !== undefined) {
            ros.close();
        }
        setTimeout(pollROS, shortDelay);
    });

    ros.on('close', function() {
        //console.log('Connection to websocket server closed.');
        console.log('ROSLIB Websocket closed');
        connectedToROS = false;
        //updateConnectedButton();
        setTimeout(pollROS, shortDelay);
    });
};

pollROS();
*/
console.log('arloBehavior.js is done, behold the power of async!');
