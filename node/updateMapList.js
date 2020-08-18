const webModel = require('./webModel');
const getMapList = require('./getMapList');
// Set map list based on file names in the map folder
const mapDir = `${process.env.HOME}/.arlobot/rosmaps/`;

module.exports = () => {
  webModel.mapList = [];
  getMapList(mapDir, (err, data) => {
    data.forEach((value) => {
      webModel.mapList.push(value.replace('.data', ''));
    });
  });
};
