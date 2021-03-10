/*
 * This is the "behavior" of the robot.
 * It should just be a very simple flow of function calls.
 *
 * This was once a fancy "Behavior" system using a library,
 * but the operation was complex and it required outside tools to view and modify.
 *
 * This aims to use simple program logic, so it should be clear if you read the code,
 * what the robot will do,
 * and you can just edit the code to change things.
 *
 * The only "rule" is to try to avoid putting any code here other than the logic and function calls.
 *
 */

const polling = require('./behaviors/polling');
const startROS = require('./behaviors/startROS');
const makeMap = require('./behaviors/makeMap');
const loadMap = require('./behaviors/loadMap');
const unPlugRobot = require('./behaviors/unPlugRobot');
const goToWaypoint = require('./behaviors/goToWaypoint');
const handlePowerWithoutROS = require('./behaviors/handlePowerWithoutROS');
const pickRandomWaypoint = require('./behaviors/pickRandomWaypoint');

const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
const wait = require('./wait');
const killROS = require('./killROS');

async function loop() {
  if (webModel.debugging && webModel.logBehaviorMessages) {
    console.log('Behave loop starting:');
  }
  // This is a bit of a catchall.
  // Certainly some things here could be broken out into their own behavior files.
  await polling();

  // The pattern is that if a behavior is "working on something" it returns false,
  // causing the loop to abort, so that it can keep working uninterrupted.
  // Otherwise it should always return true, to let the loop do the next thing.

  // Each behavior should 100% contain its own logic to decide when
  // and when not to run it.
  // This script will call EVERY behavior, EVERY time.

  // A behavior should be callable over and over, even if it is already running.

  // The order is only important in that entries higher in the stack
  // get checked first, and hence get to act and short circuit
  // the loop before lower entries.

  const behaviorFunctions = [
    startROS,
    makeMap,
    loadMap,
    unPlugRobot,
    goToWaypoint,
    pickRandomWaypoint,
    handlePowerWithoutROS,
  ];

  for (const behavior of behaviorFunctions) {
    // eslint-disable-next-line no-await-in-loop
    if (!(await behavior())) {
      return;
    }
  }

  if (webModel.debugging && webModel.logBehaviorMessages) {
    console.log(' Nothing stopped this behave loop.');
  }
}

async function behave() {
  while (!webModel.shutdownRequested) {
    try {
      // eslint-disable-next-line no-await-in-loop
      await loop();
    } catch (e) {
      console.error('Behavior Loop Error:');
      console.error(e);
    }
    // eslint-disable-next-line no-await-in-loop
    await wait(1);
  }
  // This allows the script to kill itself.
  if (!webModel.killRosHasRun) {
    console.log('Shutdown Requested via webModel.');
  }
  webModelFunctions.behaviorStatusUpdate('Shutdown Requested via webModel.');
  killROS(true);
}

module.exports = behave;
