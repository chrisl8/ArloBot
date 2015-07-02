window.onload = function() {

    // For LCARish buttons:
    var LCARishButton = function(buttonID, knockoutItem, offAction, onAction) {
        var buttonElement = document.getElementById(buttonID),
            onSide = buttonElement.getElementsByClassName("switch-on")[0],
            offSide = buttonElement.getElementsByClassName("switch-off")[0],
            switchItself = buttonElement.getElementsByClassName("toggle")[0],
            oldValue = knockoutItem();
        if (knockoutItem()) {
            onSide.classList.add("toggler--is-active");
            offSide.classList.remove("toggler--is-inactive");
            switchItself.classList.add("toggle-on");
            switchItself.classList.remove("toggle-off");
        } else {
            offSide.classList.add("toggler--is-inactive");
            onSide.classList.remove("toggler--is-active");
            switchItself.classList.remove("toggle-on");
            switchItself.classList.add("toggle-off");
        }
        knockoutItem.subscribe(function(newValue) {
            if (newValue !== oldValue) {
                oldValue = newValue;
                if (newValue) {
                    onAction();
                } else {
                    offAction();
                }
            }
            if (newValue) {
                onSide.classList.add("toggler--is-active");
                offSide.classList.remove("toggler--is-inactive");
                switchItself.classList.add("toggle-on");
                switchItself.classList.remove("toggle-off");
            } else {
                offSide.classList.add("toggler--is-inactive");
                onSide.classList.remove("toggler--is-active");
                switchItself.classList.remove("toggle-on");
                switchItself.classList.add("toggle-off");
            }
        });
        onSide.addEventListener("click", function() {
            oldValue = true;
            onAction();
        });
        offSide.addEventListener("click", function() {
            oldValue = false;
            offAction();
        });
    };

    var robotModel = {
        /*
        robotStatus: ko.observable('Robot web server is not running.'),
        pluggedIn: ko.observable('AC Status unknown.'),
        ROSisRunning: ko.observable(false),
        inRoom: ko.observable(false),
        showStartROSbutton: ko.observable(true),
        showStopROSbutton: ko.observable(true),
        mapList: ko.observableArray(['Explore!']),
        */
        selectedMap: ko.observable(),
        showSaveMap: function() {
            mapNameDialogVisible(true);
        },
        startROS: function() {
            socket.emit('startROS');
        },
        stopROS: function() {
            socket.emit('stopROS');
        },
        halt: function() {
            socket.emit('haltRobot');
        },
        unHalt: function() {
            socket.emit('unHaltRobot');
        },
        quiet: function() {
            socket.emit('beQuiet');
        },
        talk: function() {
            socket.emit('talk');
        },
        pauseAutoExplore: function() {
            socket.emit('pauseAutoExplore');
        },
        unPauseAutoExplore: function() {
            socket.emit('unPauseAutoExplore');
        },

        monitorAC: function() {
            socket.emit('monitorAC');
        },
        ignoreAC: function() {
            socket.emit('ignoreAC');
        },

        monitorIR: function() {
            socket.emit('monitorIR');
        },
        ignoreIR: function() {
            socket.emit('ignoreIR');
        },

        monitorCliff: function() {
            socket.emit('monitorCliff');
        },
        ignoreCliff: function() {
            socket.emit('ignoreCliff');
        },

        monitorProximity: function() {
            socket.emit('monitorProximity');
        },
        ignoreProximity: function() {
            socket.emit('ignoreProximity');
        },

        newMapName: ko.observable(),
        saveMap: function() {
            socket.emit('saveMap', webModel.newMapName());
        },
        //TODO: This doesn't update live. webserver.sh needs to update this live somehow.
        markBasementClosed: function() {
            socket.emit('markBasementClosed');
        },
        startLogStreamer: function() {
            socket.emit('startLogStreamer');
        },
        stopLogStreamer: function() {
            socket.emit('stopLogStreamer');
        },
        buttonOneText: ko.observable('Curiosity'),
        buttonOne: function() {
            this.buttonOneText('Cat is Dead');
        },
        buttonTwoText: ko.observable('Button'),
        buttonTwo: function() {
            this.buttonTwoText('Please do not push this button again!');
        },
        buttonThreeText: ko.observable('Button'),
        buttonThree: function() {
            this.buttonThreeText("Now you've done it!");
        },
        buttonFourText: ko.observable('Button'),
        buttonFour: function() {
            this.buttonFourText('<sigh>');
        },
        reloadPage: function() {
            document.location.reload(true);
        },
        exitWebServer: function() {
            socket.emit('exit');
        },
        remoteControlPage: function() {
            window.open("/");
        },
        openLogStreamer: function() {
            var logStreamerURL = 'http://' + location.hostname + ':28778/';
            console.log(logStreamerURL);
            window.open(logStreamerURL, '_blank');
        }
    };

    var webModel = ko.mapping.fromJS(robotModel);
    var firstStart = true;

    robotModel.selectedMap.subscribe(function(newValue) {
        socket.emit('setMap', newValue);
    });

    //var sensor = document.getElementById('sensor');
    //var socket = io.connect('http: //arlobot:8080');
    var socket = io();

    socket.on('startup', function(data) {
        ko.mapping.fromJS(data, webModel);
        if (firstStart) {
            firstStart = false;

            // Run the function once for each button:
            //LCARishButton('button-id', koWatchItem, koOffAction, koOnAction)
            LCARishButton('talk-bequiet-button', webModel.beQuiet, webModel.talk, webModel.quiet);
            LCARishButton('explore-pause-button', webModel.pauseExplore, webModel.unPauseAutoExplore, webModel.pauseAutoExplore);
            LCARishButton('ignore-pluggedIn-button', webModel.ignorePluggedIn, webModel.monitorAC, webModel.ignoreAC);
            LCARishButton('ignore-IR-button', webModel.rosParameters.ignoreIRSensors, webModel.monitorIR, webModel.ignoreIR);
            LCARishButton('ignore-cliff-button', webModel.rosParameters.ignoreCliffSensors, webModel.monitorCliff, webModel.ignoreCliff);
            LCARishButton('ignore-proximity-button', webModel.rosParameters.ignoreProximity, webModel.monitorProximity, webModel.ignoreProximity);

            ko.applyBindings(webModel);
        }
        if (webModel.autoExplore()) webModel.selectedMap('Explore!');
        else webModel.selectedMap(webModel.mapName());
    });

    socket.on('webModel', function(data) {
        console.log('.');
        //console.log(webModel); // For debugging
        ko.mapping.fromJS(data, webModel);
        if (webModel.autoExplore()) {
            webModel.selectedMap('Explore!');
        } else if (webModel.mapName() != '') {
            if (webModel.selectedMap !== webModel.mapName()) {
                webModel.selectedMap(webModel.mapName());
            }
        }
    });


    socket.on('disconnect', function() {
        webModel.selectedMap('');
        webModel.status('Robot web server disconnected.');
        webModel.scrollingStatus('Start ~/catkin_ws/src/Metatron/scripts/behavior.sh');
    });
};
