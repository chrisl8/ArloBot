const webModel = require('../webModel');
const webModelFunctions = require('../webModelFunctions');
const robotModel = require('../robotModel');

async function goToWaypoint() {
  if (webModel.debugging && webModel.logBehaviorMessages) {
    const message = ' - Checking: Go to Waypoint';
    console.log(message);
    webModelFunctions.scrollingStatusUpdate(message);
  }
  // ROS Process launch behavior pattern:
  // FIRST: Is the process already started?
  if (robotModel.goToWaypointProcess.started) {
    // startupComplete indicates either:
    // Script exited
    // Script threw "success string"
    // Script returned any data if it wasn't given a "success string"
    // If your process just runs forever if it is "GOOD" then it will not
    // exit until we are DONE, and it should "SU"
    // but if it is a run, wait for finish, succeed type script,
    // we should RUNNING until .hasExited
    // NOTE:
    // Seriously understand this so you can know where to return
    // RUNNING <-
    // SUCCESS <- Not usually even used by "RUN, WAIT, RETURN" behaviors,
    //          but returned every loop by PERPETUAL behavior scripts.
    // FAILURE <- Usually means we aren't do things.
    if (robotModel.goToWaypointProcess.startupComplete) {
      if (robotModel.goToWaypointProcess.hasExited) {
        // Once the process has exited:
        // 1. DISABLE whatever user action causes it to be called,
        // so that it won't loop.
        webModelFunctions.updateWayPointNavigator(
          'mostRecentArrival',
          webModel.wayPointNavigator.wayPointName,
        );
        webModelFunctions.updateWayPointNavigator('goToWaypoint', false);
        // 2. Now that it won't loop, set .started to false,
        // so that it can be run again.
        robotModel.goToWaypointProcess.started = false;
        // 3. Send a status to the web site:
        if (robotModel.goToWaypointProcess.exitCode > 0) {
          webModelFunctions.update('lastNavigationResult', 'Failure');
        }
        webModelFunctions.update('navigationInProgress', false);
        // 4. Log the closure to the console,
        // because this is significant.
        webModelFunctions.scrollingStatusUpdate(
          'Go to Waypoint Process Closed.',
        );
        // 5. Set any special status flags for this
        // process. i.e. ROSisRunning sets the start/stop button position
        //  NONE
        // 6. Any special "cleanup" required?
        //  NONE
        // Leave it 'RUNNING' and
        // let the next Behavior tick respond as it would,
        // if this function was never requested.
        return false;
      }
      if (!webModel.wayPointNavigator.goToWaypoint) {
        // KILL a node here if you want it to STOP!
        // Otherwise this is a non-event,
        // Either way the response should probably be RUNNING.
        // IF we were told NOT to run, we need to stop the process,
        // and then wait for the failure to arrive here on the next loop.
        // Insert command to stop current function here:
        // This command must be OK with being called multiple times.
        //  TODO: If we need to kill the goToWayPoint, do it here.
        // Don't change anything else,
        // Let the next loop fall into the "hasExited" option above.c
        if (webModel.debugging && webModel.logBehaviorMessages) {
          console.log('Go to Waypoint: RUNNING');
          webModelFunctions.scrollingStatusUpdate('Go to Waypoint: RUNNING');
        }
        return false;
      }
      // LOOK HERE!
      // If this is a "PERPETUAL" process, then this is where you want
      // to return "SUCCESS" because "startupComplete" is true,
      // but it has not exited!
      // If this is a "RUN, WAIT, RETURN" process, then this is where
      // you want to return RUNNING to let behavior tree know
      // that we are IN PROCESS!
      // This is where we go if the start is complete,
      // and did not fail.
      // and we still want it running.
      // This will repeat on every tick!
      // 1. Set any special status flags for this
      // process. i.e. ROSisRunning sets the start/stop button position
      //  NONE for GoToWaypoint
      // Whether we return 'RUNNING' or 'SUCCESS',
      // is dependent on how this Behavior node works.
      // GoToWaypoint should exit when the task is done,
      // so we call this running:
      if (webModel.debugging && webModel.logBehaviorMessages) {
        console.log('Go to Waypoint: RUNNING');
        webModelFunctions.scrollingStatusUpdate('Go to Waypoint: RUNNING');
      }

      if (robotModel.goToWaypointProcess.finalSuccess) {
        robotModel.goToWaypointProcess.finalSuccess = false;
        // TODO: Talk about it.
        webModelFunctions.update('lastNavigationResult', 'Success');
        webModelFunctions.update(
          'status',
          `Arrived at ${webModel.wayPointNavigator.wayPointName}`,
        );
      }

      if (robotModel.goToWaypointProcess.finalFailure) {
        robotModel.goToWaypointProcess.finalFailure = false;
        // TODO: Talk about it.
        webModelFunctions.update('lastNavigationResult', 'Failure');
      }

      return false;
    }
    webModelFunctions.behaviorStatusUpdate('Go to Waypoint Starting up . . .');
    console.log('Go to Waypoint: RUNNING');
    return false;
  }
  if (
    webModel.ROSisRunning &&
    !webModel.pluggedIn &&
    webModel.wayPointNavigator.goToWaypoint
  ) {
    // IF the process is supposed to start, but wasn't,
    // then run it:
    webModelFunctions.update('lastNavigationResult', false);
    webModelFunctions.update(
      'status',
      `Going to waypoint ${webModel.wayPointNavigator.wayPointName}`,
    );
    webModelFunctions.update('navigationInProgress', true);

    robotModel.goToWaypointProcess.scriptArguments = [
      robotModel.wayPointNavigator.destinationWaypoint,
    ];

    robotModel.goToWaypointProcess.start();
    webModelFunctions.scrollingStatusUpdate('Go to Waypoint Process starting!');
    return false;
  }
  // This behavior is idle, allow behave loop to continue to next entry.
  return true;
}

module.exports = goToWaypoint;
