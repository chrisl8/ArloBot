const getMapList = require('./getMapList.js');

const mapDir = `${process.env.HOME}/.arlobot/rosmaps/`;
const formatOutput = (err, data) => {
  data.forEach((value) => {
    console.log(value.replace('.data', ''));
  });
};
getMapList(mapDir, formatOutput);
