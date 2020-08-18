const exec = require('child_process').exec;

const robotModel = require('./robotModel');
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
const LCD = require('./LCD');

function killROS(exitWhenDone) {
  LCD({ operation: 'color', red: 255, green: 0, blue: 0 });
  LCD({ operation: 'clear' });
  LCD({
    operation: 'text',
    input: ' ! Killing ROS !',
    row: 'top',
  });
  if (exitWhenDone) {
    if (robotModel.semaphoreFilesWatcher) {
      robotModel.semaphoreFilesWatcher
        .close()
        .then(() => console.log('Semaphore File Watcher closed.'));
    }
    LCD({
      operation: 'text',
      input: '  and Exiting...',
      row: 'bottom',
    });
  }
  const command = `${__dirname}/../scripts/kill_ros.sh`;
  // It is rather catastrophic if this repeats!
  if (!webModel.killRosHasRun) {
    webModelFunctions.update('killRosHasRun', true);
    webModelFunctions.update('ROSstart', false);
    // ROS drops all knowledge when restarted,
    // So make it clear to the user that he needs to pick a map again.
    webModelFunctions.update('mapName', '');
    webModelFunctions.update('mapLoaded', false);
    webModelFunctions.scrollingStatusUpdate('Running kill_ros.sh . . .');
    // Logging to console too, because feedback on shutdown is nice.
    console.log('Running kill_ros.sh . . .');
    // and then also run the kill ROS command:
    const shutdownCommand = exec(command);
    shutdownCommand.stdout.on('data', (data) => {
      webModelFunctions.scrollingStatusUpdate(`Shutdown: ${data}`);
      console.log(`Shutdown:${data.toString().replace(/[\n\r]/g, '')}`);
    });

    shutdownCommand.stderr.on('data', (data) => {
      webModelFunctions.scrollingStatusUpdate(`Shutdown: ${data}`);
      console.log(`Shutdown:${data.toString().replace(/[\n\r]/g, '')}`);
    });

    shutdownCommand.on('close', (code) => {
      webModelFunctions.scrollingStatusUpdate(
        `kill_ros.sh closed with code ${code}`,
      );
      console.log(`kill_ros.sh closed with code ${code}`);
      if (exitWhenDone) {
        process.exit();
      } else {
        webModel.killRosHasRun = false;
        webModelFunctions.update('ROSisRunning', false);
      }
    });
    shutdownCommand.on('error', (err) => {
      webModelFunctions.scrollingStatusUpdate(`Shutdown process error${err}`);
    });
  }
}

module.exports = killROS;
