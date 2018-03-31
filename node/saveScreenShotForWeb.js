const exec = require('child_process').exec;
const fs = require('fs');
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
const personalData = require('./personalData');

function resolvePath(str) {
    const path = require('path');
    if (str.substr(0, 2) === '~/') {
        str = (process.env.HOME || process.env.HOMEPATH || process.env.HOMEDIR || process.cwd()) + str.substr(1);
    }
    return path.resolve(str);
}

const saveScreenShotForWeb = function () {
    if (!personalData.demoWebSite) {
        const oldFileName = resolvePath(personalData.web_folder + '/xscreenOld.png');
        const newFileName = resolvePath(personalData.web_folder + '/xscreen.png');
        try {
            fs.renameSync(newFileName, oldFileName);
        } catch (err) {
            console.log("Old screenshot doens't exist yet!");
        }
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
    }
};
module.exports = saveScreenShotForWeb;

if (require.main === module) {
    saveScreenShotForWeb();
}
