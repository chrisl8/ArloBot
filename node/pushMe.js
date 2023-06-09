// Send message via Pushover API, which is free!
const Push = require('pushover-notifications');
const personalData = require('./personalData');

function pushMe(text) {
  const p = new Push({
    user: personalData.pushover.USER,
    token: personalData.pushover.TOKEN,
  });
  const msg = {
    message: text,
    // sound: personalData.pushover.sound,
    // priority: -1,
  };
  p.send(msg); // Silent with no error reporting
}

module.exports = pushMe;

if (require.main === module) {
  if (process.argv.length < 3) {
    console.log('You must provide text for the message, like this:');
    console.log(`node pushMe.js "test message"`);
    process.exit();
  }
  pushMe(process.argv[2]);
}
