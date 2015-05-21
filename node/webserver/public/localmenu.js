window.onload = function() {

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
            ko.applyBindings(webModel);
        }
        if (webModel.autoExplore()) webModel.selectedMap('Explore!');
        else webModel.selectedMap(webModel.mapName());
    });

    socket.on('webModel', function(data) {
        console.log('.');
        ko.mapping.fromJS(data, webModel);
    });


    socket.on('disconnect', function() {
        webModel.selectedMap('');
        webModel.status('Robot web server disconnected.');
    });
};
