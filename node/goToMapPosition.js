var spawn = require('child_process').spawn;

const goToMapPosition = function(position) {
    var process = spawn('unbuffer', ['../scripts/gotoMapPositionHelperScript.sh', position]);
    process.stdout.setEncoding('utf8');
    process.stdout.on('data', function(data) {
        console.log(data);
    });
    process.on('close', function(code) {
        console.log(`Done: ${code}`);
    });
    this.process.stderr.setEncoding('utf8');
    this.process.stderr.on('data', function(data) {
        console.log(data);
    });
    this.process.on('error', function(err) {
        console.log(self.name + ' error:' + err);
    });
};
module.exports = goToMapPosition;

if (require.main === module) {
    goToMapPosition(process.argv[2]);
}
