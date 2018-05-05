const ipAddress = function() {
  let ip = false;
  let firstInterface;
  // require('os').networkInterfaces().wlan0[0].address;
  const interfaces = require('os').networkInterfaces();
  for (const networkInterface in interfaces) {
    if (
      interfaces.hasOwnProperty(networkInterface) &&
      networkInterface !== 'lo' &&
      firstInterface === undefined
    ) {
      firstInterface = networkInterface;
    }
  }
  if (firstInterface) {
    ip = interfaces[firstInterface][0].address;
  }
  return ip;
};
exports.ipAddress = ipAddress;
if (require.main === module) {
  // Run the function if this is called directly instead of required.
  console.log(ipAddress());
}
