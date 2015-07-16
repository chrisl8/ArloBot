// Check if "useQRcodes" is true in  personaldata before calling this!
// because I'm not gonna check!
var spawn = require('child_process').spawn;
var webModel = require('./webModel');
var robotModel = require('./robotModel');

// http://krasimirtsonev.com/blog/article/Nodejs-managing-child-processes-starting-stopping-exec-spawn
var psTree = require('ps-tree');
var kill = function(pid, signal, callback) {
    signal = signal || 'SIGKILL';
    callback = callback || function() {};
    var killTree = true;
    if (killTree) {
        psTree(pid, function(err, children) {
            [pid].concat(
                children.map(function(p) {
                    return p.PID;
                })
            ).forEach(function(tpid) {
                try {
                    process.kill(tpid, signal);
                } catch (ex) {}
            });
            callback();
        });
    } else {
        try {
            process.kill(pid, signal);
        } catch (ex) {}
        callback();
    }
};


module.exports = function() {
    var killProcess;
    var process = spawn('../scripts/getQRcodes.sh');
    var killOnTimeout = setTimeout(function() {
        //console.log('timeout');
        kill(process.pid);
        robotModel.gettingQRcode = false;
    }, 5000);
    process.stdout.setEncoding('utf8');
    process.stdout.on('data', function(data) {
        var receivedLine = data.split('\n')[0];
        if (receivedLine !== 'Waiting for zbarcam to close . . .') {
            webModel.QRcode = receivedLine;
            //console.log(webModel.QRcode);
            kill(process.pid);
        }
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
