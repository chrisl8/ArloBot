const os = require('os');
const fetch = require('node-fetch');
const base64 = require('base-64');
const ipAddress = require('./ipAddress');
const personalData = require('./personalData');

const cloudServerConnect = async () => {
  if (personalData.cloudServer.exists) {
    const url = `${personalData.cloudServer.service}://${personalData.cloudServer.fqdn}:${personalData.cloudServer.port}/addHostname`;
    const ip = ipAddress.ipAddress();
    const hostname = os.hostname();
    const body = { hostname, ip };
    if (personalData.webServerPort && personalData.webServerPort !== 80) {
      // No point in adding port 80.
      body.port = personalData.webServerPort;
    }
    console.log(body);
    try {
      const result = await fetch(url, {
        method: 'post',
        body: JSON.stringify(body),
        headers: {
          'Content-Type': 'application/json',
          Authorization: `Basic ${base64.encode(
            `ignored:${personalData.cloudServer.password}`,
          )}`,
        },
      });

      if (result.ok) {
        console.log('Cloud Server updated, connect to local site via:');
        console.log(`${personalData.cloudServer.service}://${personalData.cloudServer.fqdn}/redirect/${hostname}`);
      } else {
        console.error('Error connecting to Cloud Server:');
        console.error(result);
      }
    } catch (e) {
      console.error('Error connecting to Cloud Server:');
      console.error(e);
    }
  }
};

module.exports = cloudServerConnect;
