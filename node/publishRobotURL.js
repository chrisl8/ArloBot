var personalData = require('./personalData');
var webModel = require('./webModel');
var webModelFunctions = require('./webModelFunctions');
var request = require('request');
var ipAddress = require('./ipAddress');
var updateRobotURL = function() {
    var robotIP = ipAddress.ipAddress();
    var robotURL = 'http://' + robotIP + ':' + personalData.webServerPort + '/index2.html';
    if (personalData.cloudServer.exists && webModel.robotURL !== robotURL) {
        webModelFunctions.update('robotIP', robotIP);
        webModelFunctions.update('robotURL', robotURL);
        console.log('webModel.robotIP:', webModel.robotIP);
        console.log('webModel.robotURL:', webModel.robotURL);

        var serverURL = 'http://' + personalData.cloudServer.fqdn + ':' + personalData.cloudServer.port + '/updateRobotURL';

        request.post(
            serverURL,
            { json: {
                password: personalData.cloudServer.password,
                localURL: robotURL
            } },
            function (error, response, body) {
                if (!error && response.statusCode == 200) {
                    console.log(body)
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
