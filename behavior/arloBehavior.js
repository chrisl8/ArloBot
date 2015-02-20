// http://behavior3js.guineashots.com/
// Setup:
// npm install behavior3js
// npm install pushover-notifications
// npm install say
// Modifications:
// node_modules/behavior3js/package.json:
//  -"main": "libs/b3core.0.1.0.min.js",
//  +"main": "libs/b3core.0.1.0.js",
// node_modules/behavior3js/libs/b3core.0.1.0.js:
//   this.b3 = this.b3 || {};
//  +b3 = {};

var behavior3js = require('behavior3js');

var exec = require('child_process').exec;
var spawn = require('child_process').spawn;
var push = require('pushover-notifications');
// Note that tts will convert text to speech,
// or it will send a ".wav" string (path, etc)
// to aplay.
// The benefit of using it is that it will honor
// system wide "bequiet" requests.
// The same modele is used by ROS Python scripts.
var tts = require('../scripts/tts.js');

var textme = require('../scripts/textme.js');
var arloTree = new b3.BehaviorTree();

var ROSisRunning = b3.Class(b3.Condition);
ROSisRunning.prototype.name = 'ROSisRunning';
ROSisRunning.prototype.tick = function(tick) {
    // Stub tick
    console.log("ROSisRunning");
    //TODO: IF ROS is running, then set:
    //metatronProcessStarted
    //to false, so that if it stops running,
    //StartROS will start it again.
    //TODO: This will be ased on ROS subscriptions.
    //TODO: somewhere we need to subjscribe jsut like arloweb!
    return b3.FAILURE;
};

var StartROS = b3.Class(b3.Action);
StartROS.prototype.name = 'StartROS';
StartROS.prototype.tick = function(tick) {
    console.log("Start ROS");
    if (arloBot.metatronProcessStarted) {
        if (arloBot.metatronStartupNoError) {
            if (arloBot.metatronProcessStartupComplete) {
                console.log("SUCCESS!");
                return b3.SUCCESS;
            } else {
                console.log("RUNNING!");
                return b3.RUNNING;
            }
        } else {
            console.log("FAILURE");
            return b3.FAILURE;
        }
    } else {
        console.log("Running child process . . .");
        arloBot.metatronProcessStartupComplete = false;
        arloBot.metatronProcessStarted = true;
        // node suffers from similar user environment issues
        // as running via PHP from the web,
        // so PHP's start script works well,
        // although we can run it as me, so no need for the
        // stub to call with sudo
        // TODO: Make this path more portable!

        var rosProcess = spawn('../../arloweb/startROS.sh');

        rosProcess.stdout.setEncoding('utf8');
        rosProcess.stdout.on('data', function(data) {
            //console.log('stdout: ' + data);
            // We can search the output for text:
            // if (data.indexOf('SUCCESS: ROS Started') > -1) {
            //     arloBot.metatronProcessStartupComplete = true;
            // }

            // This is what we get if ROS exits, such as
            // if killed externally.
            if (data.indexOf('[master] killing on exit') > -1) {
                arloBot.metatronStartupNoError = false;
                var p = new push({
                    user: personalData.pushover.USER,
                    token: personalData.pushover.TOKEN
                });
                var msg = {
                    message: "ROS has Shut Down.",
                    title: "ROS Shutdown!",
                    sound: personalData.pushover.sound,
                    priority: 1 // Most will be -1 or 0, but this is important
                };
                p.send(msg); // Silent with no error reporting
                tts('What happend?');
                // TODO: At this point we may want to set
                // arloBot.metatronProcessStarted = false;
                // And let it try to restart?
            }
        });
        // rosProcess.stderr.setEncoding('utf8');
        // rosProcess.stderr.on('data', function (data) {
        //     console.log('stderr: ' + data);
        // });
        rosProcess.on('exit', function(code) {
            // Will catch multiple exit codes I think:
            //console.log('ROS Startup exit with code: ' + code);
            if (code === 0) {
                // SUCCESS
                arloBot.metatronStartupNoError = true;
                arloBot.metatronProcessStartupComplete = true;
            } else {
                // FAILURE
                arloBot.metatronStartupNoError = false;
                var p = new push({
                    user: personalData.pushover.USER,
                    token: personalData.pushover.TOKEN
                });
                var msg = {
                    message: "ROS Startup Failed.",
                    title: "ROS Startup ERROR!",
                    sound: personalData.pushover.sound,
                    priority: 1 // Most will be -1 or 0, but this is important
                };
                p.send(msg); // Silent with no error reporting
                tts('It hates me.');
                // TODO: TEST, and how do we send failure info
                // to Pushover and say?
            }
        });

        // This doesn't work because it never returns if the ROS process
        // works and stays alive! We have to use spawn.
        // exec('../../arloweb/startROS.sh', function(error, stdout) {
        //     if (error.code > 0) {
        //         arloBot.metatronStartupNoError = false;
        //         arloBot.metaTronStartupERROR = stdout;
        //         // Report ERROR:
        //         // TODO: Should this be its own behavior?
        //         var p = new push({
        //             user: personalData.pushover.USER,
        //             token: personalData.pushover.TOKEN
        //         });
        //         var msg = {
        //             message: arloBot.metaTronStartupERROR,
        //             title: "ROS Startup ERROR!",
        //             sound: personalData.pushover.sound,
        //             priority: 1 // Most will be -1 or 0, but this is important
        //         };
        //         p.send(msg); // Silent with no error reporting
        //         // NOTE: say uses Festival,
        //         // which is great, because there is no good interface for it,
        //         // but getting the voice you want can be a pain!
        //         // no callback, fire and forget
        //         // "null" means use system default
        //         say.speak(null, 'It hates me.');
        //     } else {
        //         arloBot.metatronProcessStartupComplete = true;
        //     }
        // });
        return b3.RUNNING;
    }
};

var RobotKnowsWhatRoomItIsIn = b3.Class(b3.Condition);
RobotKnowsWhatRoomItIsIn.prototype.name = 'RobotKnowsWhatRoomItIsIn';
RobotKnowsWhatRoomItIsIn.prototype.tick = function(tick) {
    console.log(this.name);
    return b3.FAILURE;
};

var DetermineRoom = b3.Class(b3.Action);
DetermineRoom.prototype.name = 'DetermineRoom';
DetermineRoom.prototype.tick = function(tick) {
    // Response will be handled by metatron_listener service
    // And put into topics,
    // We only need to send out the request here,
    // and only once.
    if (!arloBot.whereamiTextSent) {
        textme('Where am I?');
    }
    arloBot.whereamiTextSent = true;
    return b3.SUCCESS;
};

var UnPlugRobot = b3.Class(b3.Action);
UnPlugRobot.prototype.name = 'UnPlugRobot';
UnPlugRobot.prototype.tick = function(tick) {
    console.log(this.name);
    return b3.RUNNING;
};

var LaptopBatteryCharged = b3.Class(b3.Action);
LaptopBatteryCharged.prototype.name = 'LaptopBatteryCharged';
LaptopBatteryCharged.prototype.tick = function(tick) {
    console.log(this.name);
    return b3.FAILURE;
};

var RobotIsUnplugged = b3.Class(b3.Condition);
RobotIsUnplugged.prototype.name = 'RobotIsUnplugged';
RobotIsUnplugged.prototype.tick = function(tick) {
    console.log(this.name);
    return b3.FAILURE;
};

var MyAction2 = b3.Class(b3.Action);
MyAction2.prototype.name = 'MyAction2';
MyAction2.prototype.tick = function(tick) {
    console.log(this.name);
    return b3.FAILURE;
};

// Build this file with http://behavior3js.guineashots.com/editor/#
var fs = require('fs');
// and you can LOAD this data into the editor to start where you left off again
var arloNodeData = JSON.parse(fs.readFileSync('arloTreeData.json', 'utf8'));

// Despite the Editor creating a beautify JSON behavior tree for us,
// we still have to list the custom nodes for it by hand.
//var customNodeNames = {
//'ROSisRunning': ROSisRunning,
//'StartROS': StartROS,
//'UnPlugRobot': UnPlugRobot,
//'LaptopBatteryCharged': LaptopBatteryCharged,
//'RobotIsUnplugged': RobotIsUnplugged
//};
// TODO: Does this work as the better way to do this?
var customNodeNames = {};

function parseCustomNodes(element, index, array) {
    customNodeNames[element.name] = eval(element.name);
}
arloNodeData.custom_nodes.forEach(parseCustomNodes);
//console.log(customNodeNames);

arloTree.load(arloNodeData, customNodeNames);

// Load personal settings not included in git repo
var personalDataFile = process.env.HOME + '/.arlobot/personalDataForBehavior.json';
var personalData = JSON.parse(fs.readFileSync(personalDataFile, 'utf8'));

// This is the Arlobot "object" we will use to track everything,
// note that there is also a "blackboard" where nodes can track
// their internal status, even amongst multiple calls
// to the same node type,
// but this will keep track of braoder robot settings.
// NOTE: Do NOT track things here that the nodes
// should be collecting from real time status!
// Rather track things we can only know from inside of here,
// such as last action or speech time, etc.
var arloBot = {
    metatronProcessStarted: false,
    metatronStartupNoError: true,
    metatronProcessStartupComplete: false,
    metatronStartupERROR: '',
    whereamiTextSent: false
};

var blackboard = new b3.Blackboard();

console.log('start');
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
    //console.log(blackboard);
    console.log("---");
}, 1000);
//tree.tick(arloBot,blackboard);
console.log('done');
