var webModel = require('./webModel');
var webModelFunctions = require('./webModelFunctions');
var robotModel = require('./robotModel');
// TODO: Observe is DEAD, so this all has to change,
// Until then this entire module is dead.
//var O = require('observed');
//var webModelWatcher = O(webModel);
//var robotModelWatcher = O(robotModel);
var tts = require('./tts');
var howManySecondsSince = require('./howManySecondsSince');
// fs.watch sees like 5 updates instead of one,
// chokidar is smarter,
// although it is still possible to hit the file when it is empty,
// thus the timeout and the
// fancy error checking in the on('change' callback
var chokidar = require('chokidar');
// For random selection of items
var Stochator = require('stochator');
var fs = require('fs');
var speechModel = JSON.parse(fs.readFileSync('./speechModel.json', 'utf8'));
var speechInMemoryObject = {};
var eventModel = JSON.parse(fs.readFileSync('./eventResponses.json', 'utf8'));
var eventInMemoryObject = {};
var lastSpoke = new Date();

function buildInMemoryObject(model, memoryModel) {
    for (var item in model) {
        if (model.hasOwnProperty(item)) {
            //console.log(item);
            // If this is the first round,
            // or if this item was added since the last round,
            // populate the in memory speech model status.
            if (!memoryModel.hasOwnProperty(item)) {
                memoryModel[item] = {
                    lastSaid: null
                };
            }
            // Always reinitialize the random chooser in case any bits of it were changed
            memoryModel[item].biasedTextChooser = undefined;
        }
    }

}
buildInMemoryObject(speechModel, speechInMemoryObject);

function buildInMemoryEventObject(model, memoryModel) {
    for (var item in model) {
        if (model.hasOwnProperty(item)) {
            //console.log(item);
            // If this is the first round,
            // or if this item was added since the last round,
            // populate the in memory speech model status.
            if (!memoryModel.hasOwnProperty(item)) {
                memoryModel[item] = {};
            }
            for (var innerItem in model[item]) {
                if (innerItem !== 'repeatInterval' && innerItem !== 'delay') {
                    memoryModel[item][innerItem] = {
                        lastSaid: null
                    };
                    // Always reinitialize the random chooser in case any bits of it were changed
                    memoryModel[item][innerItem].biasedTextChooser = undefined;
                }
            }
        }
    }
}
buildInMemoryEventObject(eventModel, eventInMemoryObject);

// Watch the speechModel.json file for changes,
// So that we can dynamically add speech without restarting the robot.
chokidar.watch('./speechModel.json').on('change', function (event, path) {
    setTimeout(function () {
        fs.readFile('./speechModel.json', 'utf8', function (err, data) {
            if (err) {
                console.log('speechModel.json read error');
            } else {
                try {
                    speechModel = JSON.parse(data);
                    buildInMemoryObject(speechModel, speechInMemoryObject);
                } catch (e) {
                    return console.error(e);
                }
            }
        });
    }, 1000);
});

chokidar.watch('./eventResponses.json').on('change', function (event, path) {
    setTimeout(function () {
        fs.readFile('./eventResponses.json', 'utf8', function (err, data) {
            if (err) {
                console.log('eventResponses.json read error');
            } else {
                try {
                    eventModel = JSON.parse(data);
                    buildInMemoryEventObject(eventModel, eventInMemoryObject);
                } catch (e) {
                    return console.error(e);
                }
            }
        });
    }, 1000);
});

// Create an in memory random picker for each speech model entry.
function initializeBiasedTextChooser(model, memoryModel, item) {
    'use strict';
    var values = [],
        weights = [];
    // Repeat items based on "instance" variable.
    for (var text in model[item].thingsToSay) {
        // "repeat" is a setting, not an item to say.
        if (text !== 'repeat') {
            for (var i = 0; i < model[item].thingsToSay[text].instance; i++) {
                values.push(model[item].thingsToSay[text].text);
                weights.push(model[item].thingsToSay[text].weight);
            }
        }
    }
    // replacement must be false for eventResponses.json
    memoryModel[item].biasedTextChooser = new Stochator({
        kind: "set",
        values: values,
        weights: weights,
        replacement: model[item].thingsToSay.repeat
    });
}

function getSomethingToSay(model, memoryModel, item) {
    'use strict';
    // Initialize the generator if it is not already.
    if (memoryModel[item].biasedTextChooser === undefined) {
        initializeBiasedTextChooser(model, memoryModel, item);
    }
    var returnValue = memoryModel[item].biasedTextChooser.next();
    var count = 0;
    while (returnValue === undefined && count < 10) {
        count++; // to prevent an infinite loop due to some failure
        initializeBiasedTextChooser(model, memoryModel, item);
        returnValue = memoryModel[item].biasedTextChooser.next();
    }
    return returnValue;
}

// speechModel will hold update-able text and sound file references.

// This function can either be called by the "poll",
// or just be set up in a setInterval loop
function talkToMe() {
    for (var speechItem in speechModel) {
        if (speechModel.hasOwnProperty(speechItem)) {
            // Check if we should say this
            try {
                if (eval(speechModel[speechItem].Test)) {
                    // Check the delay to for this item, this sets how much delay must be between this item and ANY previous speech
                    if (howManySecondsSince(lastSpoke) >= speechModel[speechItem].delay) {
                        // and if we've already said it too recently we should say something else or skip it.
                        if (howManySecondsSince(speechInMemoryObject[speechItem].lastSaid) >= speechModel[speechItem].repeatInterval) {
                            //TODO: Do we need a "blank" entry, so that sometimes it says nothing, but still counts?
                            tts(getSomethingToSay(speechModel, speechInMemoryObject, speechItem));
                            lastSpoke = new Date();
                            speechInMemoryObject[speechItem].lastSaid = new Date();
                            break; // Now that we said something we are done, start over on the next loop.
                        }
                    }
                }
            } catch (e) {
                console.log('You broke your speech engine: ' + e);
            }
        }
    }
}

function talkAboutEvents(key, value) {
    if (key && value) {
        if (webModel.debugging) {
            console.log('---------------')
            console.log('talkAboutEvents change:');
            console.log(key);
            console.log(value);
            console.log('---------------')
        }
        if (eventModel[key] && howManySecondsSince(lastSpoke) >= eventModel[key].delay) {
            // and if we've already said it too recently we should say something else or skip it.
            if (howManySecondsSince(eventInMemoryObject[key][value].lastSaid) >= eventModel[key].repeatInterval) {
                tts(getSomethingToSay(eventModel[key], eventInMemoryObject[key], value));
                lastSpoke = new Date();
                eventInMemoryObject[key][value].lastSaid = new Date();
            }
        }
    }
}

// TODO: Observe is DEAD, so this all has to change,
// Until then this entire module is dead.
// TODO: Implement or improve some sort of setter/getter on webModel and robotModel
// that can be used to "ping" things like this.
webModelFunctions.emitter.on('change', function (key, value) {
    talkAboutEvents(key, value)
});
//webModelWatcher.on('change', talkAboutEvents);
//robotModelWatcher.on('change', talkAboutEvents);

// TODO: We need to track waypoint arrivals.

// TODO: For testing watchers:

//setInterval(function() {
//    if (webModel.pluggedIn === false) {
//        webModel.pluggedIn = true;
//    } else {
//        webModel.pluggedIn = false;
//    }
//}, 2500);

//setInterval(function() {
//    if (robotModel.fullyCharged === false) {
//        robotModel.fullyCharged = true;
//    } else {
//        robotModel.fullyCharged = false;
//    }
//}, 2500);

// TODO: Is this a good time?
// TODO: or should it be in the index.js Behavior tree poll?
setInterval(function () {
    talkToMe();
}, 1000);
