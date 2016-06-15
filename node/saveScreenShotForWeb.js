const exec = require('child_process').exec;
const fs = require('fs');
var webModel = require('./webModel');
var webModelFunctions = require('./webModelFunctions');
var personalData = require('./personalData');

function resolvePath(str) {
    const path = require('path');
    if (str.substr(0, 2) === '~/') {
        str = (process.env.HOME || process.env.HOMEPATH || process.env.HOMEDIR || process.cwd()) + str.substr(1);
    }
    return path.resolve(str);
}

const saveScreenShotForWeb = function () {
    const oldFileName = resolvePath(personalData.web_folder + '/xscreenOld.png');
    const newFileName = resolvePath(personalData.web_folder + '/xscreen.png');
    fs.renameSync(newFileName, oldFileName);
    const command = "DISPLAY=:0.0 /usr/bin/import -window root " + newFileName;
    exec(command, () => { // Argument options: error, stdout, stderr
        const compare = "compare -metric RMSE " + newFileName + " " + oldFileName + " /dev/null";
        exec(compare, (error, stdout, stderr) => {
            if (stderr) {
                const output = stderr.split(' ')[0];
                if (output[0]) {
                    if (output[0] > 0 && !webModel.cameraOn) {
                        webModelFunctions.update('videoSource', 'xscreen.png?_ts=' + new Date().getTime());
                    }
                }
            }
        })
    });
};
module.exports = saveScreenShotForWeb;

if (require.main === module) {
    saveScreenShotForWeb();
}
