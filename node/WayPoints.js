const getCurrentPosition = require('./getCurrentPosition');
const fs = require('fs');
const mkdirp = require('mkdirp');
const personalDataFolder = process.env.HOME + '/.arlobot/';
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');

class WayPoints {

  getWayPoint(name, callback) {
    const waypointFolder = personalDataFolder + 'waypoints/' + webModel.mapName + '/';
    const wayPointFile = waypointFolder + name;
    fs.readFile(wayPointFile, 'utf8', (err, data) => {
      if (err) {
        console.error('Error getting waypoint.');
      } else {
        callback(data);
      }
    });
  };

  async createWayPoint(name) {
    webModelFunctions.scrollingStatusUpdate(`Creating a waypoint with name ${name}`);
    const position = await getCurrentPosition();
    if (position) {
      this.saveWayPoint(position, name);
    } else {
      console.error('Error setting waypoint.');
      webModelFunctions.scrollingStatusUpdate(`Error setting waypoint ${name}`);
    }
  };

  saveWayPoint(position, name) {
    const waypointFolder = personalDataFolder + 'waypoints/' + webModel.mapName + '/';
    const wayPointFile = waypointFolder + name;
    mkdirp(waypointFolder, 0o777, function (err) {
      if (err) {
        console.error("{\"STATUS\": \"ERROR\" }");
        console.error("Could not create " + waypointFolder);
      } else {
        fs.writeFile(wayPointFile, position, (err) => {
          if (err) {
            webModelFunctions.scrollingStatusUpdate(`ERROR writing waypoint ${name} to disk`);
            console.error(`ERROR writing waypoint ${name} to disk:`);
            console.error(err);
          }
          webModelFunctions.scrollingStatusUpdate(`Waypoint ${name} has been written to disk`);
        });
      }
    });
  };

  updateWayPointList() {
    const waypointFolder = personalDataFolder + 'waypoints/' + webModel.mapName + '/';
    fs.readdir(waypointFolder, (err, list) => {
      if (err) {
        webModelFunctions.scrollingStatusUpdate('Error updating waypoint list.');
        console.error('Error updating waypoint list:');
        console.error(err);
      } else {
        webModel.wayPoints = list;
      }
    });
  };
}

module.exports = WayPoints;
