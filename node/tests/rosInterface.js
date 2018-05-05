const robotModel = require('../robotModel');
const webModel = require('../webModel');
const rosInterface = require('../rosInterface');

rosInterface.start();
setInterval(() => {
  console.log(robotModel);
  console.log(webModel.rosParameters);
  console.log(webModel.rosParameters.explorePaused);
}, 5000);
