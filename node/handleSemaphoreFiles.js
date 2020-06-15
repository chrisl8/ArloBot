// This code places 'semaphore' files into the file system
// that the ROS Python code and other node apps
// watch and respond to.
// This is called by the Poll tree function,
// so if index.js or the Behavior Tree hang up, this won't happen,
// but then the robot won't be good for much at that point anyway.
// I'm just pointing out that this doesn't actively monitor anything,
// it is called in a polling loop by index.js
const chokidar = require('chokidar');
const fs = require('fs');
const { promisify } = require('util');

const access = promisify(fs.access);
const chmod = promisify(fs.chmod);
const readdir = promisify(fs.readdir);
const mkdirp = require('mkdirp');
const webModelFunctions = require('./webModelFunctions');
const webModel = require('./webModel');
const robotModel = require('./robotModel');

const personalDataFolder = `${process.env.HOME}/.arlobot/`;
const statusFolder = `${personalDataFolder}status/`;
const quietFile = `${statusFolder}bequiet`;
const stopFile = `${statusFolder}STOP`;
const checkMasterRelayFile = `${statusFolder}checkMasterRelay`;
const checkUsbRelayBankFile = `${statusFolder}checkUsbRelayBank`;
const doorFileFolder = `${statusFolder}doors`;
// Note this will only work if we do not KNOW what map we are on.
const doorFile = `${statusFolder}/doors/unknown-door`;
const foldersExist = {
  statusFolder: false,
  doorFileFolder: false,
};

function getFileNameFromFullPath(path) {
  const splitName = path.split('/');
  return splitName[splitName.length - 1];
}

function getFolderNameFromFullPath(path) {
  const splitName = path.split('/');
  return splitName[splitName.length - 2];
}

async function folderExists(folderName) {
  while (!foldersExist[folderName]) {
    try {
      // eslint-disable-next-line no-await-in-loop
      await access(folderName);
      // eslint-disable-next-line no-await-in-loop
      await chmod(folderName, 0o777);
      foldersExist[folderName] = true;
    } catch (e) {
      if (e.errno && e.errno === -2) {
        // eslint-disable-next-line no-await-in-loop
        await mkdirp(folderName, 0o777);
        console.log(`Created status folder ${folderName}`);
      } else {
        console.error(`Error accessing semaphore folder ${folderName}:`);
        console.error(e);
      }
    }
  }
}

// Ensure the status folder exists
folderExists(statusFolder);
folderExists(doorFileFolder);

async function checkDoorFiles() {
  // Read folder if any change happens and set based on existence of any files
  try {
    const doorFileList = await readdir(doorFileFolder);
    if (doorFileList.length > 0) {
      webModelFunctions.update('doorsOpen', true);
    } else {
      webModelFunctions.update('doorsOpen', false);
    }
  } catch (e) {
    console.error(`Door folder problem: ${e}`);
    // True on error for safety.
    webModelFunctions.update('doorsOpen', true);
  }
}

checkDoorFiles();

const setFileValue = async (path, action) => {
  const fileName = getFileNameFromFullPath(path);
  const folderName = getFolderNameFromFullPath(path);
  let deleteFile = false;
  if (fileName === 'STOP') {
    webModelFunctions.update('haltRobot', action === 'add');
  } else if (fileName === 'bequiet') {
    webModelFunctions.update('beQuiet', action === 'add');
  } else if (fileName === 'checkMasterRelay') {
    webModelFunctions.update('checkMasterRelay', true);
    deleteFile = true;
  } else if (fileName === 'checkUsbRelayBank') {
    webModelFunctions.update('checkUsbRelayBank', true);
    deleteFile = true;
  } else if (folderName === 'doors') {
    checkDoorFiles();
  }
  if (deleteFile) {
    fs.unlink(path, (err) => {
      if (err && err.code !== 'ENOENT') console.error(err);
      if (webModel.debugging) {
        console.log(`successfully deleted ${path}`);
      }
    });
  }
};

// Use chokidar to watch files rather than polling for changes
function startSemaphoreFileWatcher() {
  webModelFunctions.update('semaphoreFilesRead', true);

  robotModel.semaphoreFilesWatcher = chokidar.watch(statusFolder, {
    persistent: true,
  });

  robotModel.semaphoreFilesWatcher
    .on('add', (path) => setFileValue(path, 'add'))
    .on('unlink', (path) => setFileValue(path, 'unlink'));
}

async function folderExistsForText(text) {
  let folderName = statusFolder;
  if (text === 'markDoorsClosed') {
    folderName = doorFileFolder;
  }

  await folderExists(folderName);
}

const setSemaphoreFiles = async (text) => {
  await folderExistsForText(text);

  if (text === 'talk') {
    webModelFunctions.update('beQuiet', false);
    fs.unlink(quietFile, (err) => {
      if (err && err.code !== 'ENOENT') console.error(err);
      if (webModel.debugging) {
        console.log(`successfully deleted beQuiet file`);
      }
    });
  } else if (text === 'beQuiet') {
    webModelFunctions.update('beQuiet', true);
    fs.writeFile(quietFile, 'quiet\n', (err) => {
      if (err) {
        console.error('Error writing beQuiet file:');
        console.error(err);
        foldersExist[statusFolder] = false;
      }
    });
  } else if (text === 'go') {
    webModelFunctions.update('haltRobot', false);
    fs.unlink(stopFile, (err) => {
      if (err && err.code !== 'ENOENT') console.error(err);
      if (webModel.debugging) {
        console.log(`successfully deleted haltRobot file`);
      }
    });
  } else if (text === 'stop') {
    webModelFunctions.update('haltRobot', true);
    fs.writeFile(stopFile, 'STOP\n', (err) => {
      if (err) {
        console.error('Error writing haltRobot file:');
        console.error(err);
        foldersExist[statusFolder] = false;
      }
    });
  } else if (text === 'markDoorsClosed') {
    // Wipe out ALL door files if asked!
    // The "right" way is to test the doors,
    // but that will leave files that will prevent
    // exploring, and will keep the warning
    // button on even if robot will go.
    fs.readdir(doorFileFolder, (err, files) => {
      if (err) {
        console.error('Error clearing door files:');
        console.error(err);
        foldersExist[doorFileFolder] = false;
      } else {
        files.forEach((file) => {
          fs.unlink(`${doorFileFolder}/${file}`, (innerError) => {
            if (innerError && innerError.code !== 'ENOENT')
              console.error(innerError);
            if (webModel.debugging) {
              console.log(`successfully deleted ${doorFileFolder}/${file}`);
            }
          });
        });
      }
    });
  } else if (text === 'markDoorsOpen') {
    fs.writeFile(doorFile, 'STOP\n', (err) => {
      if (err) {
        console.error('Error writing haltRobot file:');
        console.error(err);
        foldersExist[statusFolder] = false;
      }
    });
  } else if (text === 'checkMasterRelayFile') {
    fs.writeFile(checkMasterRelayFile, text, (err) => {
      if (err) {
        console.error(`Error writing ${text} file:`);
        console.error(err);
        foldersExist[statusFolder] = false;
      }
    });
  } else if (text === 'checkUsbRelayBankFile') {
    fs.writeFile(checkUsbRelayBankFile, text, (err) => {
      if (err) {
        console.error(`Error writing ${text} file:`);
        console.error(err);
        foldersExist[statusFolder] = false;
      }
    });
  }
};

exports.setSemaphoreFiles = setSemaphoreFiles;
exports.startSemaphoreFileWatcher = startSemaphoreFileWatcher;
