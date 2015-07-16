// Check if "useQRcodes" is true in  personaldata before calling this!
// because I'm not gonna check!
var spawn = require('child_process').spawn;
var webModel = require('./webModel');
var robotModel = require('./robotModel');

module.exports = function() {
    var killProcess;
    var process = spawn('../scripts/getQRcodes.sh');
    var killOnTimeout = setTimeout(function() {
        //console.log('timeout');
        killProcess = spawn('pkill', ['zbarcam']);
        robotModel.gettingQRcode = false;
    }, 5000);
    process.stdout.setEncoding('utf8');
    process.stdout.on('data', function(data) {
        webModel.QRcode = data.split('\n')[0];
        //console.log(webModel.QRcode);
        killProcess = spawn('pkill', ['-f', 'zbarcam']);
    });
    //process.stderr.setEncoding('utf8');
    //process.stderr.on('data', function(data) {
    //    console.log('stderr data:' + data);
    //});
    //process.on('error', function(err) {
    //    console.log('getQRcodes Error:' + err);
    //});
    process.on('exit', function() {
        clearTimeout(killOnTimeout);
        robotModel.gettingQRcode = false;
    });
};
