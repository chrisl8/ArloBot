var ipAddress = function () {
    var ip = false;
    var firstInterface = undefined;
    // require('os').networkInterfaces().wlan0[0].address;
    var interfaces = require('os').networkInterfaces();
    for (var interface in interfaces) {
        if (interfaces.hasOwnProperty(interface) && interface !== 'lo' && firstInterface === undefined) {
            firstInterface = interface;
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
