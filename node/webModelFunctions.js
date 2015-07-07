var webModel = require('./webModel');

var scrollingStatusUpdate = function(input) {
    webModel.scrollingStatus = input + '<br/>' + webModel.scrollingStatus;
};

var behaviorStatusUpdate = function(input) {
    webModel.behaviorStatus = input;
};

exports.scrollingStatusUpdate = scrollingStatusUpdate;
exports.behaviorStatusUpdate = behaviorStatusUpdate;
