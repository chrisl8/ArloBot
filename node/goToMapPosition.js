var spawn = require('child_process').spawn;

module.exports = function(position) {
    var process = spawn('unbuffer', ['rosservice', 'call', '/arlobot_goto/go_to_goal', position]);
    process.stdout.setEncoding('utf8');
    process.stdout.on('data', function(data) {
        console.log(data);
    });
    process.on('exit', function(code) {
        console.log('Done.');
    });
    this.process.stderr.setEncoding('utf8');
    this.process.stderr.on('data', function(data) {
        console.log(data);
    });
    this.process.on('error', function(err) {
        console.log(self.name + ' error:' + err);
    });
};
