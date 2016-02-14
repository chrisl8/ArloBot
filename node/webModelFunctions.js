// These are functions intended to manipulate the
// webModel object.
// The reason they are here instead of being included in the webModel itself,
// is to keep the webModel clean for distribution to the web clients.

var webModel = require('./webModel');
// https://nodejs.org/docs/latest/api/events.html#emitter.on
const EventEmitter = require('events');
// util is required for the inheritance
const util = require('util');

function WebModelEmitter() {
    EventEmitter.call(this);
}
// Here is the inheritance of the EventEmitter "stuff":
// Without it we have no .on or .emit, etc functions.
util.inherits(WebModelEmitter, EventEmitter);

const emitter = new WebModelEmitter();
exports.emitter = emitter;

var scrollingStatusUpdate = function (value) {
    webModel.scrollingStatus = value + '<br/>' + webModel.scrollingStatus;
    emitter.emit('change');
};
exports.scrollingStatusUpdate = scrollingStatusUpdate;

var behaviorStatusUpdate = function (value) {
    if ((webModel.behaviorStatus != value)) {
        webModel.behaviorStatus = value;
        emitter.emit('change');
    }
};
exports.behaviorStatusUpdate = behaviorStatusUpdate;

var update = function (key, value) {
    if ((webModel[key] != value)) {
        webModel[key] = value;
        emitter.emit('change', key, value);
    }
};
exports.update = update;
