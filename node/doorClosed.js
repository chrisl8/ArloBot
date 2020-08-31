const fetch = require('node-fetch');
const https = require('https');
const personalData = require('./personalData');

// To disable SSL verification for ths self-signed cert
// https://stackoverflow.com/a/59944400/4982408
const httpsAgent = new https.Agent({
  rejectUnauthorized: false,
});

const doorClosed = async (specificDoor) => {
  if (personalData.doorServerIP) {
    let doorName;
    if (specificDoor) {
      doorName = specificDoor;
    } else {
      doorName = personalData.doorName;
    }
    const url = `https://${personalData.doorServerIP}/httpendpoint/?label=${doorName}`;
    try {
      const response = await fetch(url, {
        method: 'get',
        agent: httpsAgent,
      });
      if (response.ok) {
        const json = await response.json();
        if (
          json &&
          json.hasOwnProperty('metrics') &&
          json.metrics.hasOwnProperty('Status')
        ) {
          const doorStatus = json.metrics.Status.value;
          const updateTime = Math.floor(json.metrics.Status.updated / 1000);
          const currentTime = Math.floor(Date.now() / 1000);
          // For debugging
          // console.log(
          //   doorStatus,
          //   updateTime,
          //   currentTime,
          //   currentTime - updateTime,
          // );
          if (doorStatus === 'CLOSED' && currentTime - updateTime < 2) {
            return true;
          }
          return false;
        }
      }
      console.error('Unable to obtain door status.');
      console.error(response.status, response.statusText);
    } catch (e) {
      console.error('Unable to obtain door status.');
      console.error(e);
    }
  }
  return false;
};

if (require.main === module) {
  // Run the function if this is called directly instead of required.
  let specificDoor;
  if (process.argv.length > 2) {
    specificDoor = process.argv[2];
  }
  // eslint-disable-next-line func-names
  (async function () {
    const returnData = await doorClosed(specificDoor);
    console.log(returnData);
  })();
}
