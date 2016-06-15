var personalData = require('./personalData');
var webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
var request = require('request');
var ipAddress = require('./ipAddress');
var updateRobotURL = function () {
    const robotIP = ipAddress.ipAddress();
    const robotURL = 'http://' + robotIP + ':' + personalData.webServerPort;
    if (personalData.cloudServer.exists && webModel.robotURL !== robotURL) {
        webModelFunctions.update('robotIP', robotIP);

        const serverURL = 'http://' + personalData.cloudServer.fqdn + ':' + personalData.cloudServer.port + '/updateRobotURL';

        request.post(
            serverURL,
            {
                json: {
                    password: personalData.cloudServer.password,
                    localURL: robotURL
                }
            },
            function (error, response, body) {
                if (!error && response.statusCode == 200) {
                    webModelFunctions.update('robotURL', robotURL);
                    console.log(body)
                    console.log('webModel.robotIP:', webModel.robotIP);
                    console.log('webModel.robotURL:', webModel.robotURL);
                } else {
                    console.log(error, response, body);
                }
            }
        );
    }
};
exports.updateRobotURL = updateRobotURL;
if (require.main === module) {
    // Run the function if this is called directly instead of required.
    updateRobotURL();
}
