var webModel = require('../webModel');
var fs = require('fs');
var personalDataFile = process.env.HOME + '/.arlobot/personalDataForBehavior.json';
var personalData = JSON.parse(fs.readFileSync(personalDataFile, 'utf8'));
var io = require('socket.io/node_modules/socket.io-client'),
    assert = require('assert');
var personalDataFolder = process.env.HOME + '/.arlobot/';
var statusFolder = personalDataFolder + 'status/';
var quietFile = statusFolder + 'bequiet';
var stopFile = statusFolder + 'webStopRequested';
var webserver = require('../webserver');
var quietFileStatus,
    stopFileStatus;
var checkFileStatus = function(callback) {
    fs.readFile(quietFile, function(err, data) {
        if (err) quietFileStatus = false;
        if (data) quietFileStatus = true;
        fs.readFile(stopFile, function(err, data) {
            if (err) stopFileStatus = false;
            if (data) stopFileStatus = true;
            callback();
        });
    });
}
webserver.start();

describe('Suite of unit tests for webserver', function() {

    var socket;

    beforeEach(function(done) {
        // Setup
        socket = io.connect('http://localhost:' + personalData.webServerPort, {
            'reconnection delay': 0,
            'reopen delay': 0,
            'force new connection': true
        });
        socket.on('connect', function() {
            //console.log('worked...');
            done();
        });
        socket.on('event', function(data) {
            //console.log(data);
        });
        socket.on('startup', function(data) {
            //console.log('"startup" received over socket.');
        });
        socket.on('webModel', function(data) {
            //console.log('"webModel" received over socket.');
        });
        socket.on('disconnect', function() {
            //console.log('disconnected...');
        });
    });

    afterEach(function(done) {
        // Cleanup
        if (socket.connected) {
            //console.log('disconnecting...');
            socket.disconnect();
        } else {
            // There will not be a connection unless you have done() in beforeEach, socket.on('connect'...)
            //console.log('no connection to break...');
        }
        done();
    });

    describe('Check web Model for expected data', function() {

        it('should have Explore in the first mapList entry', function() {
            expect(webModel.mapList[0]).toEqual('Explore!');
        });

        it('should have maps in the mapList', function() {
            //console.log(webModel.mapList);
            expect(webModel.mapList[1]).not.toBe(undefined);
            expect(webModel.mapList[1]).not.toEqual('');
        });

    });

    describe('Remove semaphore files via socket', function() {

        beforeEach(function(done) {
            socket.emit('talk');
            socket.emit('unHaltRobot');
            setTimeout(function() {
                checkFileStatus(done)},
                1000);
        });

        it('should unSet beQuiet file and variable', function() {
            expect(webModel.beQuiet).toBe(false);
            expect(quietFileStatus).toBe(false);
        });

        it('should unSet haltRobot file and variable', function() {
            expect(webModel.haltRobot).toBe(false);
            expect(stopFileStatus).toBe(false);
        });
    });

    describe('Set semaphore files via socket', function() {

        beforeEach(function(done) {
            socket.emit('beQuiet');
            socket.emit('haltRobot');
            setTimeout(function() {
                checkFileStatus(done)},
                1000);
        });

        it('should set beQuiet file and variable', function() {
            expect(webModel.beQuiet).toBe(true);
            expect(quietFileStatus).toBe(true);
        });

        it('should set haltRobot file and variable', function() {
            expect(webModel.haltRobot).toBe(true);
            expect(stopFileStatus).toBe(true);
        });
    });

    describe('Remove semaphore files via socket again', function() {

        beforeEach(function(done) {
            socket.emit('talk');
            socket.emit('unHaltRobot');
            setTimeout(function() {
                checkFileStatus(done)},
                1000);
        });

        it('should unSet beQuiet file and variable', function() {
            expect(webModel.beQuiet).toBe(false);
            expect(quietFileStatus).toBe(false);
        });

        it('should unSet haltRobot file and variable', function() {
            expect(webModel.haltRobot).toBe(false);
            expect(stopFileStatus).toBe(false);
        });
    });

});
