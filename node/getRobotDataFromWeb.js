const personalData = require('./personalData');
const request = require('request');
const getRobotDataFromWeb = async function () {
    const serverURL = personalData.cloudServer.service + '://' + personalData.cloudServer.fqdn + ':' + personalData.cloudServer.port + '/getRobotInfo';
    return new Promise((resolve, reject) => {
        if (!personalData.cloudServer.exists) {
            reject('Robot Web Cloud Server not set up.');
        }
        request.post(
            serverURL,
            {
                json: {
                    password: personalData.cloudServer.password
                }
            },
            function (error, response) { // Arguments: error, response, body
                if (!error && response.statusCode === 200) {
                    resolve(response.body);
                } else {
                    reject('Robot URL Update failed. Check Internet connection and personalData settings.');
                }
            }
        );

    });
};
exports.getRobotDataFromWeb = getRobotDataFromWeb;

if (require.main === module) {
    // Run the function if this is called directly instead of required.
    (async function () {
        const returnData = await getRobotDataFromWeb();
        console.log(returnData);
    })();
}
