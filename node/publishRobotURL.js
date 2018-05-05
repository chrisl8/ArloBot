const personalData = require('./personalData');
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
const request = require('request');
const ipAddress = require('./ipAddress');
const os = require('os');

const robotHostname = os.hostname();
const updateRobotURL = function() {
  let robotIP = false;

  function getIpOrPublish() {
    if (!robotIP) {
      robotIP = ipAddress.ipAddress();
      setTimeout(getIpOrPublish, 30000);
    } else {
      const robotURL = `http://${robotIP}:${personalData.webServerPort}`;
      if (personalData.cloudServer.exists && webModel.robotURL !== robotURL) {
        webModelFunctions.update('robotIP', robotIP);

        const serverURL = `${personalData.cloudServer.service}://${
          personalData.cloudServer.fqdn
        }:${personalData.cloudServer.port}/updateRobotURL`;

        request.post(
          serverURL,
          {
            json: {
              password: personalData.cloudServer.password,
              localURL: robotURL,
              robotIP,
              robotHostname,
            },
          },
          (error, response) => {
            // Arguments: error, response, body
            if (!error && response.statusCode == 200) {
              webModelFunctions.update('robotURL', robotURL);
              // console.log(body)
              // console.log('webModel.robotIP:', webModel.robotIP);
              // console.log('webModel.robotURL:', webModel.robotURL);
            } else {
              console.log(
                'Robot URL Update failed. Check Internet connection and personalData settings.',
              );
              console.log(`Server URL: ${serverURL}`);
            }
          },
        );
      }
    }
  }

  getIpOrPublish();
};
exports.updateRobotURL = updateRobotURL;
if (require.main === module) {
  // Run the function if this is called directly instead of required.
  updateRobotURL();
}
