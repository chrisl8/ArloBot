const webModel = require('../webModel');
const webModelFunctions = require('../webModelFunctions');
const robotModel = require('../robotModel');
const getCmdVelIdleTime = require('../getCmdVelIdleTime');
const WayPoints = require('../WayPoints.js');

// TODO: Talk before going,
//      while going
//      after arrival.

const wayPointEditor = new WayPoints();
function pickRandomWaypoint() {
  if (webModel.debugging && webModel.logBehaviorMessages) {
    const message = ' - Checking: Random Waypoint Picker';
    console.log(message);
    webModelFunctions.scrollingStatusUpdate(message);
  }

  if (
    webModel.ROSisRunning &&
    webModel.wayPoints.length > 1 && // If we only have 1, it hardly works.
    webModel.mapName !== '' &&
    !webModel.pluggedIn &&
    !webModel.wayPointNavigator.goToWaypoint &&
    !robotModel.goToWaypointProcess.started &&
    getCmdVelIdleTime() > 2 // TODO: Set this time in a config or something.
    // TODO: Also make it 5 once we are done debugging.
  ) {
    if (webModel.debugging && webModel.logBehaviorMessages) {
      console.log(`   - Picking a random waypoint!`);
    }
    console.log(robotModel.randomWaypointList);
    if (robotModel.randomWaypointList.length === 0) {
      robotModel.randomWaypointList = [...webModel.wayPoints];
    }

    // Remove most recent waypoint from list in case this was a list reload.
    if (robotModel.randomWaypointList.length > 1) {
      const lastWaypointEntry = robotModel.randomWaypointList.indexOf(
        webModel.wayPointNavigator.wayPointName,
      );
      if (lastWaypointEntry > -1) {
        robotModel.randomWaypointList.splice(lastWaypointEntry, 1);
      }
    }

    // Pick a random entry from the list.
    const randomEntryIndex = Math.floor(
      Math.random() * robotModel.randomWaypointList.length,
    );

    // Set this as the new waypoint, just like user would via web site,
    // and remove it from our list.
    if (webModel.debugging && webModel.logBehaviorMessages) {
      console.log(`     - ${robotModel.randomWaypointList[randomEntryIndex]}`);
    }
    wayPointEditor.goToWaypoint(
      robotModel.randomWaypointList[randomEntryIndex],
    );
    robotModel.randomWaypointList.splice(randomEntryIndex, 1);

    // Preempt remaining functions, as we have an action to take now.
    return false;
  }

  // This behavior is idle, allow behave loop to continue to next entry.
  return true;
}

module.exports = pickRandomWaypoint;
