/* eslint-disable eqeqeq,no-param-reassign */
// These are functions intended to manipulate the
// webModel object.
// The reason they are here instead of being included in the webModel itself,
// is to keep the webModel clean for distribution to the web clients.

// https://nodejs.org/docs/latest/api/events.html#emitter.on
const EventEmitter = require('events');
const util = require('util');
const webModel = require('./webModel');
const robotModel = require('./robotModel');
// util is required for the inheritance
const LCD = require('./LCD');

function WebModelEmitter() {
  webModel.lastUpdateTime = Date.now();
  EventEmitter.call(this);
}

// Here is the inheritance of the EventEmitter "stuff":
// Without it we have no .on or .emit, etc functions.
util.inherits(WebModelEmitter, EventEmitter);

const emitter = new WebModelEmitter();
exports.emitter = emitter;

let previousScrollingStatusUpdate;

const scrollingStatusUpdate = (value) => {
  if (value !== previousScrollingStatusUpdate) {
    previousScrollingStatusUpdate = value;
    webModel.lastUpdateTime = Date.now();
    webModel.scrollingStatus = `${value}<br/>${webModel.scrollingStatus}`;
    emitter.emit('change');
  }
};
exports.scrollingStatusUpdate = scrollingStatusUpdate;

const behaviorStatusUpdate = (value) => {
  if (webModel.behaviorStatus != value) {
    webModel.behaviorStatus = value;
    emitter.emit('change');
    LCD({
      operation: 'text',
      input: value,
      row: 'bottom',
    });
  }
};
exports.behaviorStatusUpdate = behaviorStatusUpdate;

const update = (key, value) => {
  if (webModel[key] != value) {
    webModel[key] = value;
    emitter.emit('change', key, value);
    if (key === 'status') {
      LCD({
        operation: 'text',
        input: value,
        row: 'top',
      });
    }
  }
};
exports.update = update;

const updateRobotModel = (key, value) => {
  if (webModel[key] != value) {
    webModel[key] = value;
    emitter.emit('changeRobotModel', key, value);
  }
};
exports.updateRobotModel = updateRobotModel;

const toggle = (key) => {
  if (webModel[key] === true) {
    webModel[key] = false;
    emitter.emit('change', key, false);
  } else if (webModel[key] === false) {
    webModel[key] = true;
    emitter.emit('change', key, true);
  }
};
exports.toggle = toggle;

// Nested objects are tricky.
const updateRosParameter = (key, value) => {
  if (webModel.rosParameters[key] != value) {
    webModel.rosParameters[key] = value;
    emitter.emit('change', key, value);
  }
};
exports.updateRosParameter = updateRosParameter;

const updateRosTopicItem = (key, value) => {
  if (key === 'robotBatteryLevel') {
    value = value.toFixed(1);
  } else if (key === 'Heading' || key === 'gyroHeading') {
    value = value.toFixed(3);
  }
  const arrayIndex = webModel.rosTopicItems.findIndex((x) => x.rosName === key);
  if (
    arrayIndex !== -1 &&
    webModel.rosTopicItems[arrayIndex].status !== value
  ) {
    webModel.rosTopicItems[arrayIndex].status = value;
    // Set alert classes:
    if (
      [true, false].indexOf(webModel.rosTopicItems[arrayIndex].alertOn) === -1
    ) {
      const operators = {
        '>': function(a, b) {
          return a > b;
        },
        '<': function(a, b) {
          return a < b;
        },
        // ...
      };
      if (
        operators[webModel.rosTopicItems[arrayIndex].alertOn](
          value,
          webModel.rosTopicItems[arrayIndex].alertValue,
        )
      ) {
        webModel.rosTopicItems[arrayIndex].btnClass =
          webModel.rosTopicItems[arrayIndex].alertBtnClass;
      } else {
        webModel.rosTopicItems[arrayIndex].btnClass = '';
      }
    } else if (value === webModel.rosTopicItems[arrayIndex].alertOn) {
      webModel.rosTopicItems[arrayIndex].btnClass =
        webModel.rosTopicItems[arrayIndex].alertBtnClass;
    } else {
      webModel.rosTopicItems[arrayIndex].btnClass = '';
    }
    emitter.emit('change', key, value);
  }
};
exports.updateRosTopicItem = updateRosTopicItem;

const updateWayPointNavigator = (key, value) => {
  if (webModel.wayPointNavigator[key] != value) {
    webModel.wayPointNavigator[key] = value;
    emitter.emit('changeRobotModel', key, value);
  }
};
exports.updateWayPointNavigator = updateWayPointNavigator;

const updateRobotMasterStatus = (key, value) => {
  if (robotModel.master[key] != value) {
    robotModel.master[key] = value;
    emitter.emit('change', key, value);
  }
};
exports.updateRobotMasterStatus = updateRobotMasterStatus;

const publishRelayState = (relayNumber, relayState, relayName) => {
  if (relayName === undefined) {
    relayName = 'empty';
  }
  const relayOn = relayState === 'ON';
  const result = relayName
    .replace(/([A-Z]+)/g, ' $1')
    .replace(/([A-Z][a-z])/g, '$1');
  const fancyName = result.charAt(0).toUpperCase() + result.slice(1);
  const relayIndex = webModel.relays.findIndex((x) => x.number === relayNumber);
  if (relayIndex === -1) {
    webModel.relays.push({
      number: relayNumber,
      name: relayName,
      fancyName,
      relayOn,
    });
    emitter.emit('change', relayName, relayState);
  } else {
    if (
      relayName !== 'Empty' &&
      webModel.relays[relayIndex].name !== relayName
    ) {
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
