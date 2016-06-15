// These are functions intended to manipulate the
// webModel object.
// The reason they are here instead of being included in the webModel itself,
// is to keep the webModel clean for distribution to the web clients.

var webModel = require('./webModel');
const robotModel = require('./robotModel');
// https://nodejs.org/docs/latest/api/events.html#emitter.on
const EventEmitter = require('events');
// util is required for the inheritance
const util = require('util');

function WebModelEmitter() {
    webModel.lastUpdateTime = Date.now();
    EventEmitter.call(this);
}
// Here is the inheritance of the EventEmitter "stuff":
// Without it we have no .on or .emit, etc functions.
util.inherits(WebModelEmitter, EventEmitter);

const emitter = new WebModelEmitter();
exports.emitter = emitter;

var scrollingStatusUpdate = function (value) {
    webModel.lastUpdateTime = Date.now();
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

var updateRobotModel = function (key, value) {
    if ((webModel[key] != value)) {
        webModel[key] = value;
        emitter.emit('changeRobotModel', key, value);
    }
};
exports.updateRobotModel = updateRobotModel;

var toggle = function (key) {
    if ((webModel[key] === true)) {
        webModel[key] = false;
        emitter.emit('change', key, false);
    } else if ((webModel[key] === false)) {
        webModel[key] = true;
        emitter.emit('change', key, true);
    }
};
exports.toggle = toggle;

// Nested ojects are tricky, please suggest a better alternative to this?
var updateRosParameter = function (key, value) {
    if ((webModel.rosParameters[key] != value)) {
        webModel.rosParameters[key] = value;
        emitter.emit('change', key, value);
    }
};
exports.updateRosParameter = updateRosParameter;

var updateWayPointNavigator = function (key, value) {
    if ((webModel.wayPointNavigator[key] != value)) {
        webModel.wayPointNavigator[key] = value;
        emitter.emit('changeRobotModel', key, value);
    }
};
exports.updateWayPointNavigator = updateWayPointNavigator;

var updateRobotMasterStatus = function (key, value) {
    if ((robotModel.master[key] != value)) {
        robotModel.master[key] = value;
        emitter.emit('change', key, value);
    }
};
exports.updateRobotMasterStatus = updateRobotMasterStatus;

var publishRelayState = function (relayNumber, relayState, relayName) {
    if (relayName === undefined) {
        relayName = 'empty';
    }
    var relayOn = (relayState === 'ON');
    const result = relayName.replace(/([A-Z]+)/g, " $1").replace(/([A-Z][a-z])/g, " $1");
    const fancyName = result.charAt(0).toUpperCase() + result.slice(1);
    const relayIndex = webModel.relays.findIndex(x=> x.number === relayNumber);
    if (relayIndex === -1) {
        webModel.relays.push({
            number: relayNumber,
            name: relayName,
            fancyName: fancyName,
            relayOn: relayOn
        });
        emitter.emit('change', relayName, relayState);
    } else {
        if (relayName !== 'Empty' && webModel.relays[relayIndex].name !== relayName) {
            // Update relay name
            webModel.relays[relayIndex].name = relayName;
            webModel.relays[relayIndex].fancyName = fancyName;
        }
        if (webModel.relays[relayIndex].relayOn !== relayOn) {
            webModel.relays[relayIndex].relayOn = relayOn;
            emitter.emit('change', relayName, relayState);
        }
    }
};
exports.publishRelayState = publishRelayState;
