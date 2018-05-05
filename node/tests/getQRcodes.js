const webModel = require('../webModel');
const getQRcodes = require('../getQRcodes');
const updateMapList = require('../updateMapList');

console.log('Be patient, using timers for testing . . . ');
updateMapList();
setTimeout(() => {
  console.log(webModel);
  getQRcodes();
  setTimeout(() => {
    console.log(webModel);
  }, 3000);
}, 3000);
