var webModel = require('../webModel');
var updateMapList = require('../updateMapList');
console.log('Be patient, using timers for testing . . . ');
console.log(webModel.mapList);
updateMapList();
setTimeout(function(){console.log(webModel.mapList);}, 3000);
