// This code places 'semaphore' files into the file system
// that the ROS Python code and other node apps
// watch and respond to.
// This is called by the Poll tree function,
// so if index.js or the Behavior Tree hang up, this won't happen,
// but then the robot won't be good for much at that point anyway.
// I'm just pointing out that this doesn't actively monitor anything,
// it is called in a polling loop by index.js
const webModelFunctions = require('./webModelFunctions');
const fs = require('fs');
const {promisify} = require('util');
const access = promisify(fs.access);
const chmod = promisify(fs.chmod);
const readFile = promisify(fs.readFile);
const readdir = promisify(fs.readdir);
const mkdirp = require('mkdirp');

const personalDataFolder = process.env.HOME + '/.arlobot/';
const statusFolder = personalDataFolder + 'status/';
const quietFile = statusFolder + 'bequiet';
const stopFile = statusFolder + 'STOP';
const doorFileFolder = statusFolder + 'doors';
// Note this will only work if we do not KNOW what map we are on.
const doorFile = statusFolder + '/doors/unknown-door';
const foldersExist = {
    statusFolder: false,
    doorFileFolder: false
};

async function folderExists(folderName) {
    while (!foldersExist[folderName]) {
        try {
            await access(folderName);
            await chmod(folderName, 0o777);
            foldersExist[folderName] = true;
        } catch (e) {
            if (e.errno && e.errno === -2) {
                await mkdirp(folderName, 0o777);
                console.log(`Created status folder ${folderName}`);
            } else {
                console.error(`Error accessing semaphore folder ${folderName}:`);
                console.error(e);
            }
        }
    }
}

async function folderExistsForText(text) {
    let folderName = statusFolder;
    if (text === 'markDoorsClosed') {
        folderName = doorFileFolder;
    }

    await folderExists(folderName);
}

const setSemaphoreFiles = async text => {

    await folderExistsForText(text);

    if (text === 'talk') {
        webModelFunctions.update('beQuiet', false);
        fs.unlink(quietFile, readSemaphoreFiles);
    } else if (text === 'beQuiet') {
        webModelFunctions.update('beQuiet', true);
        fs.writeFile(quietFile, 'quiet\n', err => {
            if (err) {
                console.error('Error writing beQuiet file:');
                console.error(err);
                foldersExist[statusFolder] = false;
            }
        });
    } else if (text === 'go') {
        webModelFunctions.update('haltRobot', false);
        fs.unlink(stopFile, readSemaphoreFiles);
    } else if (text === 'stop') {
        webModelFunctions.update('haltRobot', true);
        fs.writeFile(stopFile, 'STOP\n', err => {
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
                files.forEach(function (file) {
                    fs.unlink(doorFileFolder + '/' + file, readSemaphoreFiles);
                });
            }
        });
    } else if (text === 'markDoorsOpen') {
        fs.writeFile(doorFile, 'STOP\n');
    }
};

const readSemaphoreFiles = async () => {

    const checkFileAndSetValue = async (file, value) => {
        try {
            await readFile(file, 'utf8');
            webModelFunctions.update(value, true);
        } catch (e) {
            webModelFunctions.update(value, false);
        }
    };

    await folderExists(statusFolder);
    await checkFileAndSetValue(stopFile, 'haltRobot');
    await checkFileAndSetValue(quietFile, 'beQuiet');

    // Check door files
    // TODO: How can we tell if the folder only has files in it for the wrong map?
    await folderExists(doorFileFolder);
    try {
        const doorFileList = await readdir(doorFileFolder);
        if (doorFileList.length > 0) {
            webModelFunctions.update('doorsOpen', true);
        } else {
            webModelFunctions.update('doorsOpen', false);
        }
    } catch (e) {
        console.error('Door folder problem: ' + err);
        // True on error for safety.
        webModelFunctions.update('doorsOpen', true);
    }

    webModelFunctions.update('semaphoreFilesRead', true);
};

exports.setSemaphoreFiles = setSemaphoreFiles;
exports.readSemaphoreFiles = readSemaphoreFiles;
