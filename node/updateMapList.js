const webModel = require('./webModel');
const getMapList = require('./getMapList');
// Set map list based on file names in the map folder
const mapDir = `${process.env.HOME}/.arlobot/rosmaps/`;

module.exports = () => {
  webModel.mapList = ['Explore!'];
  getMapList(mapDir, (err, data) => {
    data.forEach((value) => {
      webModel.mapList.push(value.replace('.yaml', ''));
    });
  });
};
