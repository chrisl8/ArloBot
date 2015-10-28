var robotModel = require('../robotModel');
var webModel = require('../webModel');
var rosInterface = require('../rosInterface');
rosInterface.start();
setInterval(function() {
    console.log(robotModel);
    console.log(webModel.rosParameters);
    console.log(webModel.rosParameters.explorePaused);
}, 5000);
