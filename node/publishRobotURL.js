const os = require('os');
const personalData = require('./personalData');
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
const ipAddress = require('./ipAddress');

const robotHostname = os.hostname();
const updateRobotURL = async () => {
  let robotIP = false;

  async function getIpOrPublish() {
    if (!robotIP) {
      robotIP = ipAddress.ipAddress();
      setTimeout(getIpOrPublish, 30000);
    } else {
      const robotURL = `http://${robotIP}:${personalData.webServerPort}`;
      /** @namespace personalData.cloudServer */
      if (personalData.cloudServer.exists && webModel.robotURL !== robotURL) {
        webModelFunctions.update('robotIP', robotIP);

        /** @namespace personalData.cloudServer.fqdn */
        const serverURL = `${personalData.cloudServer.service}://${personalData.cloudServer.fqdn}:${personalData.cloudServer.port}/updateRobotURL`;

        const body = {
          password: personalData.cloudServer.password,
          localURL: robotURL,
          robotIP,
          robotHostname,
        };
        try {
          const response = await fetch(serverURL, {
            method: 'post',
            body: JSON.stringify(body),
            headers: { 'Content-Type': 'application/json' },
          });
          if (response.ok) {
            console.log('RobotWebService updated.');
            webModelFunctions.update('robotURL', robotURL);
            return;
          }
          console.error(
            'Robot URL Update failed. Check Internet connection and personalData settings.',
          );
          console.error(`Server URL: ${serverURL}`);
          console.error(response.status, response.statusText);
        } catch (e) {
          console.error(
            'Robot URL Update failed. Check Internet connection and personalData settings.',
          );
          console.error(`Server URL: ${serverURL}`);
          console.error(e);
        }
      }
    }
  }

  await getIpOrPublish();
};
exports.updateRobotURL = updateRobotURL;

if (require.main === module) {
  // Run the function if this is called directly instead of required.
  // eslint-disable-next-line no-unused-expressions,func-names
  (async function () {
    await updateRobotURL();
  });
}
