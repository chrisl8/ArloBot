const fs = require('fs');
const mkdirp = require('mkdirp');
const getCurrentPosition = require('./getCurrentPosition');

const personalDataFolder = `${process.env.HOME}/.arlobot/`;
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');

class WayPoints {
  getWayPoint(name, callback) {
    const waypointFolder = `${personalDataFolder}waypoints/${
      webModel.mapName
    }/`;
    const wayPointFile = waypointFolder + name;
    fs.readFile(wayPointFile, 'utf8', (err, data) => {
      if (err) {
        console.error('Error getting waypoint.');
      } else {
        callback(data);
      }
    });
  }

  async createWayPoint(name) {
    webModelFunctions.scrollingStatusUpdate(
      `Creating a waypoint with name ${name}`,
    );
    const position = await getCurrentPosition();
    if (position) {
      this.saveWayPoint(position, name);
    } else {
      console.error('Error setting waypoint.');
      webModelFunctions.scrollingStatusUpdate(`Error setting waypoint ${name}`);
    }
  }

  saveWayPoint(position, name) {
    const waypointFolder = `${personalDataFolder}waypoints/${
      webModel.mapName
    }/`;
    const wayPointFile = waypointFolder + name;
    mkdirp(waypointFolder, 0o777, (err) => {
      if (err) {
        console.error('{"STATUS": "ERROR" }');
        console.error(`Could not create ${waypointFolder}`);
      } else {
        fs.writeFile(wayPointFile, position, (e) => {
          if (e) {
            webModelFunctions.scrollingStatusUpdate(
              `ERROR writing waypoint ${name} to disk`,
            );
            console.error(`ERROR writing waypoint ${name} to disk:`);
            console.error(e);
          }
          webModelFunctions.scrollingStatusUpdate(
            `Waypoint ${name} has been written to disk`,
          );
        });
      }
    });
  }

  updateWayPointList() {
    const waypointFolder = `${personalDataFolder}waypoints/${
      webModel.mapName
    }/`;
    fs.readdir(waypointFolder, (err, list) => {
      if (err) {
        webModelFunctions.scrollingStatusUpdate(
          'No waypoints found for this map.',
        );
        console.log('No waypoints found for this map.');
      } else {
        webModel.wayPoints = list;
      }
    });
  }
}

module.exports = WayPoints;
