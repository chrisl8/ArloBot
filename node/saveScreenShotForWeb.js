const exec = require('child_process').exec;
const fs = require('fs');
const path = require('path');
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
const personalData = require('./personalData');

let thisSystemHasAScreen = true;

function resolvePath(str) {
  if (str.substr(0, 2) === '~/') {
    // eslint-disable-next-line no-param-reassign
    str =
      (process.env.HOME ||
        process.env.HOMEPATH ||
        process.env.HOMEDIR ||
        process.cwd()) + str.substr(1);
  }
  return path.resolve(str);
}

const saveScreenShotForWeb = () => {
  if (thisSystemHasAScreen && !personalData.demoWebSite) {
    const oldFileName = resolvePath(
      `${personalData.web_folder}/xscreenOld.png`,
    );
    const newFileName = resolvePath(`${personalData.web_folder}/xscreen.png`);
    try {
      fs.renameSync(newFileName, oldFileName);
    } catch (err) {
      console.log('Old screenshot does not exist yet!');
    }
    const command = `DISPLAY=:0.0 /usr/bin/import -silent -window root ${newFileName}`;
    exec(command, (error, stdout, stderr) => {
      if (stderr) {
        console.error(
          'This system does NOT appear to have a screen due to the following result:',
        );
        console.error(stderr);
        console.error('Suspending screenshots.');
        thisSystemHasAScreen = false;
      } else {
        // Argument options: error, stdout, stderr
        const compare = `compare -metric RMSE ${newFileName} ${oldFileName} /dev/null`;
        exec(compare, (error2, stdout2, stderr2) => {
          if (stderr2) {
            const output = stderr.split(' ')[0];
            if (output[0]) {
              if (output[0] > 0 && !webModel.cameraOn) {
                webModelFunctions.update(
                  'videoSource',
                  `xscreen.png?_ts=${new Date().getTime()}`,
                );
              }
            }
          }
        });
      }
    });
  }
};
module.exports = saveScreenShotForWeb;

if (require.main === module) {
  saveScreenShotForWeb();
}
