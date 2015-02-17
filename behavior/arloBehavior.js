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
var push = require('pushover-notifications');
var say = require('say');

var arloTree = new b3.BehaviorTree();

var ROSisRunning = b3.Class(b3.Condition);
ROSisRunning.prototype.name = 'ROSisRunning';
ROSisRunning.prototype.tick = function(tick) {
    // Stub tick
    console.log("ROSisRunning");
    //TODO: IF ROS is running, then set:
    //metaTronProcessStarted
    //to false, so that if it starts,
    //StartROS will start it again.
    return b3.FAILURE;
};

var StartROS = b3.Class(b3.Action);
StartROS.prototype.name = 'StartROS';
StartROS.prototype.tick = function(tick) {
    console.log("Start ROS");
    if (arloBot.metaTronProcessStarted) {
        if (arloBot.metatronStartupNoError) {
            console.log("RUNNING!");
            return b3.RUNNING;
        } else {
            console.log("FAILURE");
            return b3.FAILURE;
        }
    } else {
        console.log("Running child process . . .");
        arloBot.metaTronProcessStarted = true;
        // node suffers from similar user environment issues
        // as running via PHP from the web,
        // so PHP's start script works well,
        // although we can run it as me, so no need for the
        // stub to call with sudo
        // TODO: Make this path more portable!
        exec('../../arloweb/startROS.sh', function(error, stdout) {
            if (error.code > 0) {
                arloBot.metatronStartupNoError = false;
                arloBot.metaTronStartupERROR = stdout;
                // Report ERROR:
                // TODO: Should this be its own behavior?
                var p = new push({
                    user: personalData.pushover.USER,
                    token: personalData.pushover.TOKEN
                });
                var msg = {
                    message: arloBot.metaTronStartupERROR,
                    title: "ROS Startup ERROR!",
                    sound: personalData.pushover.sound,
                    priority: 1 // Most will be -1 or 0, but this is important
                };
                p.send(msg); // Silent with no error reporting
                // NOTE: say uses Festival,
                // which is great, because there is no good interface for it,
                // but getting the voice you want can be a pain!
                // no callback, fire and forget
                // "null" means use system default
                say.speak(null, 'It hates me.');
            } else {
                arloBot.metaTronProcessSuccess = true;
            }
        });
        return b3.RUNNING;
    }
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
    return b3.SUCCESS;
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
    metatronStartupERROR: '',
    // Currently filled with dummy data
    flipBit: 1,
    flipBit2: 1,
    spokeThisTick: false,
    spokeLastTick: false
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
