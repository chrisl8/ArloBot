const fs = require('fs');
const mkdirp = require('mkdirp');
const getCurrentPosition = require('./getCurrentPosition');

const personalDataFolder = `${process.env.HOME}/.arlobot/`;
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
const robotModel = require('./robotModel');

class WayPoints {
  returnToMapZeroPoint() {
    webModelFunctions.updateWayPointNavigator('wayPointName', 'Zero');
    robotModel.wayPointNavigator.destinationWaypoint =
      'pose: { position: { x: 0.0, y: 0.0, z: 0 }, orientation: { x: 0, y: 0, z: 0.0, w: 0.1} }';
    webModelFunctions.updateWayPointNavigator('goToWaypoint', true);
  }

  goToWaypoint(name) {
    if (webModel.debugging && webModel.logOtherMessages) {
      console.log(name);
    }
    if (name && webModel.wayPoints.indexOf(name) > -1) {
      webModelFunctions.updateWayPointNavigator('wayPointName', name);
      this.getWayPoint(name, (response) => {
        if (webModel.debugging && webModel.logOtherMessages) {
          console.log(response);
        }
        robotModel.wayPointNavigator.destinationWaypoint = response;
        webModelFunctions.updateWayPointNavigator('goToWaypoint', true);
      });
    }
  }

  getWayPoint(name, callback) {
    const waypointFolder = `${personalDataFolder}waypoints/${webModel.mapName}/`;
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
      await this.saveWayPoint(position, name);
    } else {
      console.error('Error setting waypoint.');
      webModelFunctions.scrollingStatusUpdate(`Error setting waypoint ${name}`);
    }
  }

  async saveWayPoint(position, name) {
    const waypointFolder = `${personalDataFolder}waypoints/${webModel.mapName}/`;
    const wayPointFile = waypointFolder + name;
    await mkdirp(waypointFolder, 0o777);
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

  updateWayPointList() {
    const waypointFolder = `${personalDataFolder}waypoints/${webModel.mapName}/`;
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
