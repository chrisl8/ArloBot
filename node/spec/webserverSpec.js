const fs = require('fs');
const io = require('socket.io-client');
const webModel = require('../webModel');

const personalDataFile = `${
  process.env.HOME
}/.arlobot/personalDataForBehavior.json`;
const personalData = JSON.parse(fs.readFileSync(personalDataFile, 'utf8'));

const personalDataFolder = `${process.env.HOME}/.arlobot/`;
const statusFolder = `${personalDataFolder}status/`;
const quietFile = `${statusFolder}bequiet`;
const stopFile = `${statusFolder}STOP`;
const webserver = require('../webserver');

let quietFileStatus, stopFileStatus;
const checkFileStatus = function(callback) {
  fs.readFile(quietFile, (err, data) => {
    if (err) {
      quietFileStatus = false;
    }
    if (data) {
      quietFileStatus = true;
    }
    fs.readFile(stopFile, (err, data) => {
      if (err) {
        stopFileStatus = false;
      }
      if (data) {
        stopFileStatus = true;
      }
      callback();
    });
  });
};
webserver.start();

describe('Suite of unit tests for webserver', () => {
  let socket;

  beforeEach((done) => {
    // Setup
    socket = io.connect(`http://localhost:${personalData.webServerPort}`, {
      'reconnection delay': 0,
      'reopen delay': 0,
      'force new connection': true,
    });
    socket.on('connect', () => {
      // console.log('worked...');
      done();
    });
    socket.on('event', () => {
      // console.log(data);
    });
    socket.on('startup', () => {
      // console.log('"startup" received over socket.');
    });
    socket.on('webModel', () => {
      // console.log('"webModel" received over socket.');
    });
    socket.on('disconnect', () => {
      // console.log('disconnected...');
    });
  });

  afterEach((done) => {
    // Cleanup
    if (socket.connected) {
      // console.log('disconnecting...');
      socket.disconnect();
    } else {
      // There will not be a connection unless you have done() in beforeEach, socket.on('connect'...)
      // console.log('no connection to break...');
    }
    done();
  });

  describe('Check web Model for expected data', () => {
    it('should have Explore in the first mapList entry', () => {
      expect(webModel.mapList[0]).toEqual('Explore!');
    });

    it('should have maps in the mapList', () => {
      // console.log(webModel.mapList);
      expect(webModel.mapList[1]).not.toBe(undefined);
      expect(webModel.mapList[1]).not.toEqual('');
    });
  });

  describe('Remove semaphore files via socket', () => {
    beforeEach((done) => {
      socket.emit('talk');
      socket.emit('unHaltRobot');
      setTimeout(() => {
        checkFileStatus(done);
      }, 1000);
    });

    it('should unSet beQuiet file and variable', () => {
      expect(webModel.beQuiet).toBe(false);
      expect(quietFileStatus).toBe(false);
    });

    it('should unSet haltRobot file and variable', () => {
      expect(webModel.haltRobot).toBe(false);
      expect(stopFileStatus).toBe(false);
    });
  });

  describe('Set semaphore files via socket', () => {
    beforeEach((done) => {
      socket.emit('beQuiet');
      socket.emit('haltRobot');
      setTimeout(() => {
        checkFileStatus(done);
      }, 1000);
    });

    it('should set beQuiet file and variable', () => {
      expect(webModel.beQuiet).toBe(true);
      expect(quietFileStatus).toBe(true);
    });

    it('should set haltRobot file and variable', () => {
      expect(webModel.haltRobot).toBe(true);
      expect(stopFileStatus).toBe(true);
    });
  });

  describe('Remove semaphore files via socket again', () => {
    beforeEach((done) => {
      socket.emit('talk');
      socket.emit('unHaltRobot');
      setTimeout(() => {
        checkFileStatus(done);
      }, 1000);
    });

    it('should unSet beQuiet file and variable', () => {
      expect(webModel.beQuiet).toBe(false);
      expect(quietFileStatus).toBe(false);
    });

    it('should unSet haltRobot file and variable', () => {
      expect(webModel.haltRobot).toBe(false);
      expect(stopFileStatus).toBe(false);
    });
  });
});
