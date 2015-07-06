var getCurrentPosition = require('./getCurrentPosition');
var fs = require('fs');
var mkdirp = require('mkdirp');
var personalDataFolder = process.env.HOME + '/.arlobot/';
var webModel = require('./webModel');

function WayPoints() {};

WayPoints.prototype.getWayPoint = function(name, callback) {
    var waypointFolder = personalDataFolder + 'waypoints/' + webModel.mapName + '/';
    var wayPointFile = waypointFolder + name;
    fs.readFile(wayPointFile, 'utf8', function(err, data) {
        if (err) {
            console.log('Error getting waypoint.');
        } else {
            callback(data);
        }
    });
}

WayPoints.prototype.createWayPoint = function(name) {
    getCurrentPosition(this.saveWayPoint, name);
}

WayPoints.prototype.saveWayPoint = function(position, name) {
    var waypointFolder = personalDataFolder + 'waypoints/' + webModel.mapName + '/';
    var wayPointFile = waypointFolder + name;
    mkdirp(waypointFolder, 0777, function(err) {
        if (err) {
            console.log("{\"STATUS\": \"ERROR\" }");
            console.log("Could not create " + waypointFolder);
        } else {
            fs.writeFile(wayPointFile, position);
        }
    });
}

WayPoints.prototype.updateWayPointList = function() {
    waypointFolder = personalDataFolder + 'waypoints/' + webModel.mapName + '/';
    fs.readdir(waypointFolder, function(err, list) {
        if (err) {
            console.log('Error updating waypoint list.');
        } else {
            webModel.wayPoints = list;
        }
    });
};

module.exports = WayPoints;
