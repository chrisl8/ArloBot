const fs = require('fs');

const getEtcHostsFile = async () =>
  new Promise((resolve, reject) => {
    fs.readFile('/etc/hosts', 'utf8', (err, data) => {
      if (err) {
        reject(err);
      } else {
        resolve(data);
      }
    });
  });

const writeNewEtcHostsFile = async (data) =>
  new Promise((resolve, reject) => {
    fs.writeFile('/etc/hosts', data, (err, fsData) => {
      if (err) {
        reject(err);
      } else {
        resolve(fsData);
      }
    });
  });

const addRobotIpToEtcHosts = async (robotDataString) => {
  try {
    let robotData;
    if (robotDataString) {
      robotData = JSON.parse(robotDataString);
    } else {
      // If we require this at the top, it will fail when run as root.
      // eslint-disable-next-line global-require
      const getRobotDataFromWeb = require('./getRobotDataFromWeb')
        .getRobotDataFromWeb;
      robotData = await getRobotDataFromWeb();
    }
    if (!robotData) {
      return null;
    }
    const etcHosts = await getEtcHostsFile();
    const etcHostsArray = etcHosts.split('\n');
    const newEtcHostsArray = [];
    let alreadyThere;
    etcHostsArray.forEach((line) => {
      if (line.slice(0, 1) === '#') {
        // Don't check comments, just include them.
        newEtcHostsArray.push(line);
      } else if (
        line.split(/[\s]+/)[1] &&
        line.split(/[\s]+/)[1].toLowerCase() ===
          robotData.robotHostname.toLowerCase()
      ) {
        // If the line contains the hostname, but not the right IP,
        // Then it is excluded
        if (line.split(/[\s]+/)[0] === robotData.robotIP) {
          alreadyThere = true;
          newEtcHostsArray.push(line);
        }
      } else {
        // Include everything else in the file too.
        newEtcHostsArray.push(line);
      }
    });
    if (!alreadyThere) {
      newEtcHostsArray.push(`${robotData.robotIP}\t${robotData.robotHostname}`);

      // / Put the trailing newline back
      newEtcHostsArray.push('');

      const newEtcHostsData = newEtcHostsArray.join('\n');

      await writeNewEtcHostsFile(newEtcHostsData);
    }
    return null;
  } catch (e) {
    return e;
  }
};
exports.addRobotIpToEtcHosts = addRobotIpToEtcHosts;

if (require.main === module) {
  // Run the function if this is called directly instead of required.
  (async () => {
    let robotData = null;
    if (process.argv.length > 2) {
      robotData = process.argv[2];
    }
    const error = await addRobotIpToEtcHosts(robotData);
    if (error) {
      console.log(error);
      process.exit(1);
    }
  })();
}
