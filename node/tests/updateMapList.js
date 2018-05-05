const webModel = require('../webModel');
const updateMapList = require('../updateMapList');

console.log('Be patient, using timers for testing . . . ');
console.log(webModel.mapList);
updateMapList();
setTimeout(() => {
  console.log(webModel.mapList);
}, 3000);
