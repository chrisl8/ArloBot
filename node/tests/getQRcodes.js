var webModel = require('../webModel');
var getQRcodes = require('../getQRcodes');
var updateMapList = require('../updateMapList');
console.log('Be patient, using timers for testing . . . ');
updateMapList();
setTimeout(function(){
    console.log(webModel);
    getQRcodes();
    setTimeout(function(){console.log(webModel);}, 3000);
}, 3000);
