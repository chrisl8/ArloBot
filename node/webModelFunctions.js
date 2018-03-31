// These are functions intended to manipulate the
// webModel object.
// The reason they are here instead of being included in the webModel itself,
// is to keep the webModel clean for distribution to the web clients.

const webModel = require('./webModel');
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

const scrollingStatusUpdate = function (value) {
    webModel.lastUpdateTime = Date.now();
    webModel.scrollingStatus = value + '<br/>' + webModel.scrollingStatus;
    emitter.emit('change');
};
exports.scrollingStatusUpdate = scrollingStatusUpdate;

const behaviorStatusUpdate = function (value) {
    if (webModel.behaviorStatus != value) {
    webModel.behaviorStatus = value;
    emitter.emit('change');
}
};
exports.behaviorStatusUpdate = behaviorStatusUpdate;

const update = function (key, value) {
    if (webModel[key] != value) {
    webModel[key] = value;
    emitter.emit('change', key, value);
}
};
exports.update = update;

const updateRobotModel = function (key, value) {
    if (webModel[key] != value) {
    webModel[key] = value;
    emitter.emit('changeRobotModel', key, value);
}
};
exports.updateRobotModel = updateRobotModel;

const toggle = function (key) {
    if (webModel[key] === true) {
        webModel[key] = false;
        emitter.emit('change', key, false);
    } else if ((webModel[key] === false)) {
        webModel[key] = true;
        emitter.emit('change', key, true);
    }
};
exports.toggle = toggle;

// Nested objects are tricky.
const updateRosParameter = function (key, value) {
    if (webModel.rosParameters[key] != value) {
    webModel.rosParameters[key] = value;
    emitter.emit('change', key, value);
}
};
exports.updateRosParameter = updateRosParameter;

const updateRosTopicItem = function (key, value) {
    if (key === 'robotBatteryLevel') {
        value = value.toFixed(1);
    } else if (key === 'Heading' || key === 'gyroHeading') {
        value = value.toFixed(3);
    }
    let arrayIndex = webModel.rosTopicItems.findIndex(x => x.rosName === key);
    if (arrayIndex !== -1 && (webModel.rosTopicItems[arrayIndex].status !== value)) {
        webModel.rosTopicItems[arrayIndex].status = value;
        // Set alert classes:
        if ([true, false].indexOf(webModel.rosTopicItems[arrayIndex].alertOn) === -1) {
            let operators = {
                '>': function (a, b) {
                    return a > b
                },
                '<': function (a, b) {
                    return a < b
                },
                // ...
            };
            if (operators[webModel.rosTopicItems[arrayIndex].alertOn](value, webModel.rosTopicItems[arrayIndex].alertValue)) {
                webModel.rosTopicItems[arrayIndex].btnClass = webModel.rosTopicItems[arrayIndex].alertBtnClass
            } else {
                webModel.rosTopicItems[arrayIndex].btnClass = '';
            }
        } else {
            if (value === webModel.rosTopicItems[arrayIndex].alertOn) {
                webModel.rosTopicItems[arrayIndex].btnClass = webModel.rosTopicItems[arrayIndex].alertBtnClass
            } else {
                webModel.rosTopicItems[arrayIndex].btnClass = '';
            }
        }
        emitter.emit('change', key, value);
    }
};
exports.updateRosTopicItem = updateRosTopicItem;

const updateWayPointNavigator = function (key, value) {
    if (webModel.wayPointNavigator[key] != value) {
    webModel.wayPointNavigator[key] = value;
    emitter.emit('changeRobotModel', key, value);
}
};
exports.updateWayPointNavigator = updateWayPointNavigator;

const updateRobotMasterStatus = function (key, value) {
    if (robotModel.master[key] != value) {
    robotModel.master[key] = value;
    emitter.emit('change', key, value);
}
};
exports.updateRobotMasterStatus = updateRobotMasterStatus;

const publishRelayState = function (relayNumber, relayState, relayName) {
    if (relayName === undefined) {
        relayName = 'empty';
    }
    const relayOn = (relayState === 'ON');
    const result = relayName.replace(/([A-Z]+)/g, " $1").replace(/([A-Z][a-z])/g, " $1");
    const fancyName = result.charAt(0).toUpperCase() + result.slice(1);
    const relayIndex = webModel.relays.findIndex(x => x.number === relayNumber);
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
