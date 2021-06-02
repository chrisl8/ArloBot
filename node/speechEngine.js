// For random selection of items
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
const tts = require('./tts');
const howManySecondsSince = require('./howManySecondsSince');
const speechModel = require('./speechModel').chatter;
const eventModel = require('./speechModel').events;

let lastSpoke = new Date();

// Create a random picker for each speech model entry.
function initializeBiasedTextChooser(model, item) {
  const values = [];
  const weights = [];
  const repeat = model[item].thingsToSay.repeat;
  for (const text in model[item].thingsToSay) {
    // "repeat" is a setting, not an item to say.
    if (text !== 'repeat' && model[item].thingsToSay.hasOwnProperty(text)) {
      values.push(model[item].thingsToSay[text].text);
      weights.push(model[item].thingsToSay[text].weight);
    }
  }
  // eslint-disable-next-line no-param-reassign
  model[item].biasedTextChooser = { values, weights, repeat };
}

// https://stackoverflow.com/a/55671924/4982408
function chooseWeighted(weights) {
  const sum = weights.reduce((acc, el) => acc + el, 0);
  let acc = 0;
  // eslint-disable-next-line no-return-assign
  const chances = weights.map((el) => (acc = el + acc));
  const rand = Math.random() * sum;
  return chances.filter((el) => el <= rand).length;
}

function getSomethingToSay(model, item) {
  if (
    model[item].biasedTextChooser === undefined ||
    model[item].biasedTextChooser.values.length === 0
  ) {
    if (webModel.debugging && webModel.logOtherMessages) {
      console.log(`Initializing ${item}`);
    }
    // Initialize the generator if it is not already initialized or if it is empty.
    initializeBiasedTextChooser(model, item);
  }
  if (webModel.debugging && webModel.logOtherMessages) {
    console.log(model[item].biasedTextChooser.values);
    console.log(model[item].biasedTextChooser.weights);
  }
  const returnValueIndex = chooseWeighted(
    model[item].biasedTextChooser.weights,
  );
  const returnValue = model[item].biasedTextChooser.values[returnValueIndex];
  if (!model[item].biasedTextChooser.repeat) {
    // If not set to repeat, remove used entry:
    model[item].biasedTextChooser.values.splice(returnValueIndex, 1);
    model[item].biasedTextChooser.weights.splice(returnValueIndex, 1);
  }
  return returnValue;
}

// This function can either be called by the "poll",
// or just be set up in a setInterval loop
function talkToMe() {
  for (const speechItem in speechModel) {
    if (
      speechModel.hasOwnProperty(speechItem) &&
      speechModel[speechItem].Test() &&
      howManySecondsSince(lastSpoke) >= speechModel[speechItem].spacing &&
      howManySecondsSince(speechModel[speechItem].lastSaid) >=
        speechModel[speechItem].repeatInterval
    ) {
      const textToSay = getSomethingToSay(speechModel, speechItem);
      tts(textToSay);
      lastSpoke = new Date();
      speechModel[speechItem].lastSaid = new Date();
      // Now that we said something we are done, start over on the next loop.
      // That is, we never say two things on the same loop.
      break;
    }
  }
}

// Some events just spam us, so ignore those to help mostly with the log.
const eventsToAlwaysIgnore = [
  'robot_battery_level',
  'abd_speed_limit',
  'abd_reverse_speed_limit',
  'heading',
  'min_distance_sensor',
  'myCroftSaid',
];

function talkAboutEvents(key, value) {
  if (key && value) {
    if (
      webModel.debugging &&
      webModel.logTalkAboutEvents &&
      eventsToAlwaysIgnore.indexOf(key) === -1
    ) {
      console.log('talkAboutEvents change:');
      console.log(` - ${key}: ${value}`);
      if (eventModel[key]) {
        console.log(eventModel[key]);
        console.log('-------------------');
      }
    }
    if (
      eventModel[key] &&
      howManySecondsSince(lastSpoke) >= eventModel[key].spacing &&
      eventModel[key][value] &&
      howManySecondsSince(eventModel[key][value].lastSaid) >=
        eventModel[key].repeatInterval
    ) {
      const textToSay = getSomethingToSay(eventModel[key], value);
      tts(textToSay);
      lastSpoke = new Date();
      eventModel[key][value].lastSaid = new Date();
    }
  }
}

webModelFunctions.emitter.on('change', (key, value) => {
  talkAboutEvents(key, value);
});
webModelFunctions.emitter.on('changeRobotModel', (key, value) => {
  talkAboutEvents(key, value);
});

module.exports = talkToMe;
