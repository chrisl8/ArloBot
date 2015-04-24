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

    var behavior3js = require('behavior3js');

    var exec = require('child_process').exec;
    var spawn = require('child_process').spawn;
    var ROSLIB = require('roslib');
    // Note that tts will convert text to speech,
    // or it will send a ".wav" string (path, etc)
    // to aplay.
    // The benefit of using it is that it will honor
    // system wide "bequiet" requests.
    // The same modele is used by ROS Python scripts.
    var tts = require('./tts');

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
                    }
                });
                shutdownCommand.on('error', function(err) {
                    console.log('kill_ros process error' + err);
                });
            }
    };

    function exitHandler(options, err) {
        if (options.cleanup) console.log('arloBehavior has been told to "cleanup"');
        if (err) console.log(err.stack);
        //if (options.exit) process.exit();
        if (options.exit) {
            console.log('arloBehavior has been asked to exit.');
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
    var webServer = require('./webserver/webserver');
    httpServer = webServer.start();
    var io = require("socket.io").listen(httpServer);

    var ROSrunRequested = b3.Class(b3.Condition);
    ROSrunRequested.prototype.name = 'ROSrunRequested';
    ROSrunRequested.prototype.tick = function(tick) {
        console.log(this.name);
        if (webServer.webModel.ROSstart) {
            return b3.SUCCESS;
        } else {
            if (arloBot.metatronProcessStartupComplete) {
                arloBot.metatronProcessStartupComplete = false;
                killROS(false);
            }
            return b3.FAILURE;
        }
    };

    var StartROS = b3.Class(b3.Action);
    StartROS.prototype.name = 'StartROS';
    StartROS.prototype.tick = function(tick) {
        console.log("Start ROS");
        if (arloBot.metatronProcessStarted) {
            if (arloBot.metatronStartupNoError) {
                if (arloBot.metatronProcessStartupComplete) {
                    console.log("SUCCESS!");
                    io.sockets.emit('ROSisRunning',{status: true, text: 'Running!'});
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
            io.sockets.emit('ROSisRunning',{status: false, text: 'starting up . . .'});
            // node suffers from similar user environment issues
            // as running via PHP from the web,
            // so PHP's start script works well,
            // although we can run it as me, so no need for the
            // stub to call with sudo
            // TODO: Make sure kill_ros.sh doesn't kill anything
            // that we started or need in the node app!
            var rosProcess = spawn('./startROS.sh');

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
                    tts('What happened?');
                    arloBot.metatronStartupNoError = false;
                    // Require user to request a restart
                    webServer.webModel.ROSstart = false;
                    // , and then do it.
                    arloBot.metatronProcessStarted = false;
                    io.sockets.emit('ROSisRunning',{status: false, text: 'Not Running.'});
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
                    tts('It hates me.');
                    // FAILURE
                    arloBot.metatronStartupNoError = false;
                    // Require user to request a restart
                    webServer.webModel.ROSstart = false;
                    // , and then do it.
                    arloBot.metatronProcessStarted = false;
                    io.sockets.emit('ROSisRunning',{status: false, text: 'failed to start.'});
                }
            });

            // This doesn't work because it never returns if the ROS process
            // works and stays alive! We have to use spawn.
            // exec('../../arloweb/startROS.sh', function(error, stdout) {
            //     if (error.code > 0) {
            //         arloBot.metatronStartupNoError = false;
            //         arloBot.metaTronStartupERROR = stdout;
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

    // Despite the Editor creating a beautiful JSON behavior tree for us,
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

    arloTree.load(arloNodeData, customNodeNames);

    // Load personal settings not included in git repo
    var personalDataFile = process.env.HOME + '/.arlobot/personalDataForBehavior.json';
    var personalData = JSON.parse(fs.readFileSync(personalDataFile, 'utf8'));

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
        console.log(webServer.webModel);
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
    // Copied from arloweb.js
    // Be sure to set url to point to localhost,
    // and change any references to web objects with console.log (i.e. setActionField)
    var pollROS = function() {
        console.log('pollROS run');
        connectedToROS = false;

        ros = new ROSLIB.Ros({
            url: 'ws://localhost:9090'
        });

        ros.on('connection', function() {
            console.log('Websocket connected.');
            //connectRequested = true;
            updateConnectedButton();
            checkROSServices();
        });

        ros.on('error', function(error) {
            //console.log('Error connecting to websocket server: ', error);
            console.log('Websocket eror');
            if (ros !== undefined) {
                ros.close();
            }
        });

        ros.on('close', function() {
            //console.log('Connection to websocket server closed.');
            console.log('Websocket closed');
            connectedToROS = false;
            updateConnectedButton();
            setTimeout(pollROS, shortDelay);
        });
    };

    pollROS();
    console.log('arloBehavior.js is done, behold the power of async!');
