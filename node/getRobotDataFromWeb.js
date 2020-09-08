const fetch = require('node-fetch');
const personalData = require('./personalData');

const getRobotDataFromWeb = async () => {
  if (!personalData.cloudServer.exists) {
    console.error('Robot Web Cloud Server not set up.');
    return false;
  }
  const serverURL = `${personalData.cloudServer.service}://${personalData.cloudServer.fqdn}:${personalData.cloudServer.port}/getRobotInfo`;
  const body = { password: personalData.cloudServer.password };
  try {
    const response = await fetch(serverURL, {
      method: 'post',
      body: JSON.stringify(body),
      headers: { 'Content-Type': 'application/json' },
    });
    if (response.ok) {
      return await response.json();
    }
    console.error(
      'Robot URL Update failed. Check Internet connection and personalData settings.',
    );
    console.error(response.status, response.statusText);
  } catch (e) {
    console.error(
      'Robot URL Update failed. Check Internet connection and personalData settings.',
    );
    console.error(e);
  }
  return false;
};
exports.getRobotDataFromWeb = getRobotDataFromWeb;

if (require.main === module) {
  // Run the function if this is called directly instead of required.
  // eslint-disable-next-line func-names
  (async function () {
    const returnData = await getRobotDataFromWeb();
    if (returnData) {
      if (process.argv[2] === 'json') {
        console.log(JSON.stringify(returnData));
      } else {
        console.log(returnData);
      }
    }
  })();
}
