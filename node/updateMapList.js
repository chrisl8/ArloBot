var webModel = require('./webModel');
var getMapList = require('./getMapList');
// Set map list based on file names in the map folder
var mapDir = process.env.HOME + '/.arlobot/rosmaps/';

module.exports = function() {
    webModel.mapList = ['Explore!'];
    getMapList(mapDir, function(err, data) {
        data.forEach(function(value) {
            webModel.mapList.push(value.replace('.yaml', ''));
        });
    });
};
