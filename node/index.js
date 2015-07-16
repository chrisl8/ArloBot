var webModel = require('./webModel');
var webModelFunctions = require('./webModelFunctions');
var robotModel = require('./robotModel');
var fs = require('fs');
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
var LaunchScript = require('./LaunchScript');
var textme = require('./textme');
var rosInterface = require('./rosInterface');
var webserver = require('./webserver');
var os = require('os');
var repl = require('repl');
var handleSemaphoreFiles = require('./handleSemaphoreFiles');
var getQRcodes = require('./getQRcodes');

var arloTree = new b3.BehaviorTree();

// Load personal settings not included in git repo
var personalDataFile = process.env.HOME + '/.arlobot/personalDataForBehavior.json';
var personalData = JSON.parse(fs.readFileSync(personalDataFile, 'utf8'));

rosInterface.start();

// Cleanup on shutdown
// http://stackoverflow.com/questions/14031763/doing-a-cleanup-action-just-before-node-js-exits
var kill_rosHasRun = false;
var killROS = function(exitWhenDone) {
    var command = __dirname + '/../scripts/kill_ros.sh';
    // It is rather catastrophic if this repeats!
    if (!kill_rosHasRun) {
        kill_rosHasRun = true;
        webModel.ROSstart = false;
        webModelFunctions.scrollingStatusUpdate("Running kill_ros.sh . . .");
        // Logging to console too, because feedback on shutdown is nice.
        console.log("Running kill_ros.sh . . .");
        var shutdownCommand = exec(command);
        shutdownCommand.stdout.on('data', function(data) {
            webModelFunctions.scrollingStatusUpdate('Shutdown: ' + data);
            console.log('Shutdown:' + data.toString().replace(/[\n\r]/g, ""));
        });

        shutdownCommand.stderr.on('data', function(data) {
            webModelFunctions.scrollingStatusUpdate('Shutdown: ' + data);
            console.log('Shutdown:' + data.toString().replace(/[\n\r]/g, ""));
        });

        shutdownCommand.on('close', function(code) {
            webModelFunctions.scrollingStatusUpdate('kill_ros.sh closed with code ' + code);
            console.log('kill_ros.sh closed with code ' + code);
            if (exitWhenDone) {
                process.exit();
            } else {
                kill_rosHasRun = false;
                webModel.ROSisRunning = false;
            }
        });
        shutdownCommand.on('error', function(err) {
            webModelFunctions.scrollingStatusUpdate('Shutdown process error' + err);
        });
    }
};

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
    //if (options.exit) process.exit();
    if (options.exit) {
        webModelFunctions.scrollingStatusUpdate('Main process exit, calling killROS');
        console.log('Main process exit, calling killROS');
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

webserver.start();

// ## Here are the Behavior Tree Nodes ##

var Poll = b3.Class(b3.Action);
Poll.prototype.name = 'Poll';
Poll.prototype.tick = function(tick) {
    // Some things just need to be polled, there is no way around it. Put those here.

    // Check laptop battery each tick
    var batteryCommand = '/usr/bin/upower -d|grep percentage|head -1';
    var batteryCheck = exec(batteryCommand);
    batteryCheck.stdout.on('data', function(data) {
        var re = /\s+/;
        webModel.laptopBatteryPercentage = data.split(re)[2];
        if (webModel.laptopBatteryPercentage.match('100%')) webModel.laptopFullyCharged = true;
        else webModel.laptopFullyCharged = false;
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

    handleSemaphoreFiles.readSemaphoreFiles();

    if (personalData.useQRcodes && !robotModel.gettingQRcode && robotModel.cmdTopicIdle) {
        // Old school thread control
        // It reduces how often zbarcam is run,
        // and prevents it from getting stuck
        robotModel.gettingQRcode = true;
        getQRcodes();
        // This works because this is a polling loop,
        // Otherwise it would need to be in a callback,
        // or Promise
        if (webModel.mapName === '' && webModel.mapList.indexOf(webModel.QRcode) > -1) {
            webModel.mapName = webModel.QRcode;
        }
    }

    // This node will always return success,
    // although if you want to let some polling requirement
    // hang the robot until it is done you could return running
    // further up in that call.

    return b3.SUCCESS;
};

// TODO: Perfect this pattern and replicate to all script starting behaviors.
var StartROS = b3.Class(b3.Action);
StartROS.prototype.name = 'StartROS';
StartROS.prototype.tick = function(tick) {
    // ROS Process launch behavior pattern:
    // FIRST: Is the process already started?
    if (robotModel.ROSprocess.started) {
        // startupComplete indicates either:
        // Script exited
        // Script threw "success string"
        // Script returned any data if it wasn't given a "success string"
        if (robotModel.ROSprocess.startupComplete) {
            if (robotModel.ROSprocess.hasExited) {
                // Once the process has exited:
                // 1. DISABLE whatever user action causes it to be called,
                // so that it won't loop.
                webModel.ROSstart = false;
                // 2. Now that it won't loop, set .started to false,
                // so that it can be run again.
                robotModel.ROSprocess.started = false;
                // 3. Send a status to the web site:
                webModel.status = 'ROS process has closed.';
                // 4. Log the closure to the console,
                // because this is significant.
                webModelFunctions.scrollingStatusUpdate(this.name + "Process Closed.");
                // 5. Set any special status flags for this
                // process. i.e. ROSisRunning sets the start/stop button position
                webModel.ROSisRunning = false;
                // 6. Any special "cleanup" required?
                // In this case we will run the kill routine.
                // This command must be OK with being called multiple times.
                killROS(false);
                // Leave it 'RUNNING' and
                // let the next Behavior tick respond as it would,
                // if this function was never requested.
                return b3.RUNNING;
            } else if (!webModel.ROSstart) {
                // IF we were told NOT to run, we need to stop the process,
                // and then wait for the failure to arrive here on the next loop.
                // Insert command to stop current function here:
                // This command must be OK with being called multiple times.
                killROS(false);
                // Don't change anything else,
                // Let the next loop fall into the "hasExited" option above.
                return b3.RUNNING;
            } else {
                // This is where we go if the start is complete,
                // and did not fail.
                // and we still want it running.
                // This will repeat on every tick!
                // 1. Set any special status flags for this
                // process. i.e. ROSisRunning sets the start/stop button position
                webModel.ROSisRunning = true;
                // Whether we return 'RUNNING' or 'SUCCESS',
                // is dependent on how this Behavior node works.
                return b3.SUCCESS;
            }
        } else {
            webModelFunctions.behaviorStatusUpdate(this.name + " Starting up . . .");
            return b3.RUNNING;
        }
    } else if (webModel.ROSstart) {
        // IF the process is supposed to start, but wasn't,
        // then run it:
        webModel.status = 'ROS Start Requested.';
        robotModel.ROSprocess.start();
        webModelFunctions.scrollingStatusUpdate(this.name + " Process starting!");
        return b3.RUNNING;
    }
    // If the process isn't running and wasn't requested to run:
    webModelFunctions.behaviorStatusUpdate('Waiting for StartROS request.');
    return b3.FAILURE;
};

var AutoExplore = b3.Class(b3.Action);
AutoExplore.prototype.name = 'AutoExplore';
AutoExplore.prototype.tick = function(tick) {
    if (webModel.autoExplore) {
        if (robotModel.exploreProcess.started) {

            // Catch changes in pauseExplore and send them to the arlobot_explore pause_explorer service
            if (robotModel.pauseExplore !== webModel.pauseExplore) {
                // TODO: Should this use the LaunchScript object?
                robotModel.pauseExplore = webModel.pauseExplore;
                var command = '/opt/ros/indigo/bin/rosservice call /arlobot_explore/pause_explorer ' + robotModel.pauseExplore;
                exec(command);
            }

            if (robotModel.exploreProcess.startupComplete) {
                if (robotModel.exploreProcess.hasExited) {
                    webModel.status = 'Explore process is closed.';
                    webModel.autoExplore = false;
                    robotModel.exploreProcess.started = false;
                    webModelFunctions.behaviorStatusUpdate(this.name + "FAILURE");
                    return b3.FAILURE;
                } else {
                    webModel.status = 'Explore process started.';
                    // Since this node will loop, we never reach the "unplug" node,
                    // so we need to tell the user that we are still plugged in.
                    // NOTE: This prevents self-unplugging logic,
                    // but I'm not sure we want/need that for the explore function.
                    if (webModel.pluggedIn) {
                        webModelFunctions.behaviorStatusUpdate('Robot is still plugged in!');
                        if (webModel.laptopFullyCharged) {
                            if (!robotModel.unplugMeTextSent) {
                                textme('Please unplug me!');
                                robotModel.unplugMeTextSent = true;
                            }
                        }
                    } else {
                        webModelFunctions.behaviorStatusUpdate('Robot is Exploring!');

                    }
                    return b3.RUNNING;
                }
            } else {
                webModel.status = 'Explore process is starting...';
                return b3.RUNNING;
            }
        } else {
            robotModel.exploreProcess.start();
            webModelFunctions.behaviorStatusUpdate(this.name);
            return b3.RUNNING;
        }
    } else {
        // Return FAIURE if we were NOT asked to explore,
        // thus passing priority on to LoadMap
        return b3.FAILURE;
    }
};

var LoadMap = b3.Class(b3.Action);
LoadMap.prototype.name = 'LoadMap';
LoadMap.prototype.tick = function(tick) {
    // ROS Process launch behavior pattern:
    // FIRST: This decides if we run this process or not:
    if (webModel.mapName === '') {
        if (!robotModel.whereamiTextSent) {
            textme('Where am I?');
            robotModel.whereamiTextSent = true;
        }
        // This fails if we have no map name,
        // and the tree loops again.
        return b3.FAILURE;
    } else {
        if (robotModel.loadMapProcess.started) {
            if (robotModel.loadMapProcess.startupComplete) {
                if (robotModel.loadMapProcess.hasExited) {
                    // Once the process has exited:
                    // 1. DISABLE whatever user action causes it to be called,
                    // so that it won't loop.
                    webModel.mapName = '';
                    // 2. Now that it won't loop, set .started to false,
                    // so that it can be run again.
                    robotModel.loadMapProcess.started = false;
                    // 3. Send a status to the web site:
                    webModel.status = 'Map process has closed.';
                    // 4. Log the closure to the console,
                    // because this is significant.
                    webModelFunctions.scrollingStatusUpdate(this.name + "Process Closed.");
                    // 5. Set any special status flags for this
                    // process. i.e. ROSisRunning sets the start/stop button position
                    // NOTHING HERE
                    // 6. Any special "cleanup" required?
                    // This command must be OK with being called multiple times.
                    robotModel.initialPoseSet = false;
                    // Leave it 'RUNNING' and
                    // let the next Behavior tick respond as it would,
                    // if this function was never requested.
                    return b3.RUNNING;
                } else {
                    // This will repeat on every tick!
                    if (!robotModel.initialPoseSet) {
                        // webserver.js will populate webModel.wayPoints,
                        // when the map is set.
                        if (webModel.wayPoints.indexOf('initial') > -1) {
                            robotModel.initialPoseSet = true;
                            // TODO: Set robot's initial pose based on 'initial' waypoint for map.
                        }
                    }
                    webModel.status = 'Map is Loaded.';
                    // Whether we return 'RUNNING' or 'SUCCESS',
                    // is dependent on how this Behavior node works.
                    return b3.RUNNING;
                }
            } else {
                webModelFunctions.behaviorStatusUpdate(this.name + " Starting up . . .");
                return b3.RUNNING;
            }
        } else {
            // IF the process is supposed to start, but wasn't,
            // then run it:
            webModelFunctions.scrollingStatusUpdate('Map: ' + webModel.mapName);
            //robotModel.loadMapProcess.scriptArguments = [webModel.mapName];
            robotModel.loadMapProcess.ROScommand = robotModel.loadMapProcess.ROScommand + process.env.HOME + '/.arlobot/rosmaps/' + webModel.mapName + '.yaml';
            webModelFunctions.scrollingStatusUpdate(robotModel.loadMapProcess.ROScommand);
            robotModel.loadMapProcess.start();
            return b3.RUNNING;
        }
    }
};

var UnPlugRobot = b3.Class(b3.Action);
UnPlugRobot.prototype.name = 'UnPlugRobot';
UnPlugRobot.prototype.tick = function(tick) {
    if (!webModel.pluggedIn) return b3.SUCCESS;
    if (webModel.laptopFullyCharged) {
        if (!robotModel.unplugMeTextSent) {
            textme('Please unplug me!');
            robotModel.unplugMeTextSent = true;
        } else {
            return b3.SUCCESS;
        }
    }

    // TODO: Implement self-unplugging permission and function here.

    webModelFunctions.behaviorStatusUpdate(this.name + ': FAILURE');
    // We cannot do much else until we are unplugged.
    return b3.FAILURE;
};

// Build this file with http://behavior3js.guineashots.com/editor/#
// and you can LOAD this data into the editor to start where you left off again
var arloNodeData = JSON.parse(fs.readFileSync('arloTreeData.json', 'utf8'));

// Despite the Editor creating a beautiful JSON behavior tree for us,
// we still have to list the custom nodes for it by hand like this:
//var customNodeNames = {
//'ROSisRunning': ROSisRunning,
//'StartROS': StartROS,
//'UnPlugRobot': UnPlugRobot,
//'LaptopBatteryCharged': LaptopBatteryCharged,
//'RobotIsUnplugged': RobotIsUnplugged
//};
// but I'm uisng eval to automate the build here:
var customNodeNames = {};

function parseCustomNodes(element, index, array) {
    customNodeNames[element.name] = eval(element.name); // jshint ignore:line
}
arloNodeData.custom_nodes.forEach(parseCustomNodes);

arloTree.load(arloNodeData, customNodeNames);


// ## Scripts and ROS Commands that will be called by nodes ##
// NOTE: Be sure to put 'unbuffer ' at the beginning of any ROScommand
// if you hope to monitor the output, otherwise they never flush!
// http://stackoverflow.com/a/11337310
// http://linux.die.net/man/1/unbuffer

robotModel.ROSprocess = new LaunchScript({
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
robotModel.exploreProcess = new LaunchScript({
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
robotModel.loadMapProcess = new LaunchScript({
    name: 'LoadMap',
    ROScommand: loadMapCommand,
    successString: 'odom received'
});

var blackboard = new b3.Blackboard();

webModel.status = 'Behavior Tree is running.';

// This is where the behavior tree actually runs ("loops")
setInterval(function() {
    arloTree.tick(robotModel, blackboard);
    // Note that we can do stuff between ticks if we want to,
    // although tracking information in the arloBot object,
    // or node data in the blackboard is preferable.
    //console.log(blackboard);

    // This allows the script to kill itself.
    if (webModel.shutdownRequested) {
        if (!kill_rosHasRun) {
            console.log('Shutdown Requested via webModel.');
        }
        webModelFunctions.behaviorStatusUpdate('Shutdown Requested via webModel.');
        killROS(true);
    }

}, 1000);

console.log('Go to: http://' + os.hostname() + ':' + personalData.webServerPort + '/');

// Run Firefox with this page on the local machine,
// to provide a "menu" on the robot!
if (personalData.launchBrowser) {
    var runFirefox = new LaunchScript({
        name: 'FireFox',
        scriptName: '../scripts/runFirefox.sh',
        scriptArguments: 'http://' + os.hostname() + ':' + personalData.webServerPort + '/'
    });
    runFirefox.start();
}

// REPL server at the console for command line interaction and debugging:
var replServer = repl.start({
    prompt: webModel.robotName + " > "
});

var replHelp = function() {
    console.log('Usage:\nwebModel - List webModel variables\npersonalData - List personalData contents\n.exit - Shut down robot and exit.');
}

replServer.context.webModel = webModel;
replServer.context.robotModel = robotModel;
replServer.context.personalData = personalData;
replServer.context.killROS = killROS;
replServer.context.help = replHelp;
replServer.on('exit', function() {
    console.log('Got "exit" event from repl!');
    killROS(true);
});
console.log('Press Enter to get a prompt.');
console.log('Run \'help()\' for a list of options.');
console.log('Use Ctrl+d to shut down robot.');
