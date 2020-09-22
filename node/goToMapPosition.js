const spawn = require('child_process').spawn;

const goToMapPosition = (position) => {
  const child = spawn('unbuffer', [
    '../scripts/gotoMapPositionHelperScript.sh',
    position,
  ]);
  child.stdout.setEncoding('utf8');
  child.stdout.on('data', (data) => {
    console.log(data);
  });
  child.on('close', (code) => {
    console.log(`Done: ${code}`);
  });
  this.process.stderr.setEncoding('utf8');
  this.process.stderr.on('data', (data) => {
    console.log(data);
  });
  this.process.on('error', (err) => {
    console.log(`goToMapPosition error:${err}`);
  });
};
module.exports = goToMapPosition;

if (require.main === module) {
  goToMapPosition(process.argv[2]);
}
