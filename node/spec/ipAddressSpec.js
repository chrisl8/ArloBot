const util = require('util');
const exec = util.promisify(require('child_process').exec);
const { ipAddress } = require('../ipAddress');

describe('ipAddress', () => {
  const ipAddressTypeText = /^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$/;
  it('should return an IP Address like string', () => {
    expect(ipAddress()).toMatch(ipAddressTypeText);
  });
  it('should return an IP Adress when called via command line too', async () => {
    const { stdout } = await exec('node ipAddress', {
      encoding: 'utf8',
    });
    expect(stdout.split('\n')[0]).toMatch(ipAddressTypeText);
  });
});
