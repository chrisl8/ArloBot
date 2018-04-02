const personalData = require('./personalData');
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');
const robotModel = require('./robotModel');
const tts = require('./tts');
const handleSemaphoreFiles = require('./handleSemaphoreFiles');
// This will handle messages from the remote cloud based web server.
// Presumably these will be mostly SMS messages from Twilio,
// but anything we could want to poll or push to the robot,
// via an Internet accesible web server will arrive here.

class RemoteMessageHandler {
  static parseMessageFromTwilio(message) {
    if (message.data) {
      const smsData = JSON.parse(message.data);
      if (
        smsData &&
        smsData.smsText &&
        smsData.smsFrom &&
        smsData.smsFrom === personalData.twilio.my_phone_number
      ) {
        const smsText = smsData.smsText;
        const smsTextArray = smsText.split(' ');
        const smsTextArrayLowerCase = smsTextArray.map((value) =>
          value.toLowerCase(),
        );
        console.log(`SMS Message was: ${smsText}`);
        return {
          smsText,
          smsTextArray,
          smsTextArrayLowerCase,
        };
      }
    }
    return undefined;
  }

  handleMessage(message) {
    /* Sample response:
     Handler got:
     Robot-0 { event: 'newMessage',
     Robot-0   data: '{"smsText":"Test","smsTo":"+13162851661","smsFrom":"+13162087309"}' }
     */
    const parsedTextMessage = RemoteMessageHandler.parseMessageFromTwilio(
      message,
    );
    // Here we make use of some common input:
    switch (message.event) {
      case 'newMessage':
        if (parsedTextMessage) {
          if (parsedTextMessage.smsTextArray[0].toLowerCase() === 'say') {
            const textToSay = parsedTextMessage.smsText.substr(
              parsedTextMessage.smsText.indexOf(' ') + 1,
            );
            tts(textToSay);
          } else if (
            parsedTextMessage.smsTextArray[0].toLowerCase() === 'goodnight' ||
            (parsedTextMessage.smsTextArray[0].toLocaleLowerCase() === 'good' &&
              parsedTextMessage.smsTextArray[1].toLowerCase() === 'night')
          ) {
            tts('Good night master.');
            robotModel.master.isAsleep = true;
          } else if (
            parsedTextMessage.smsTextArray[0].toLowerCase() === 'goodmorning' ||
            (parsedTextMessage.smsTextArray[0].toLocaleLowerCase() === 'good' &&
              parsedTextMessage.smsTextArray[1].toLowerCase() === 'morning')
          ) {
            tts('Top of the morning to you master!');
            robotModel.master.isAsleep = false;
          } else if (
            parsedTextMessage.smsTextArrayLowerCase.indexOf('quiet') > -1
          ) {
            tts('Shutting up now.');
            handleSemaphoreFiles.setSemaphoreFiles('beQuiet');
          } else if (
            parsedTextMessage.smsTextArrayLowerCase.indexOf('talk') > -1
          ) {
            handleSemaphoreFiles.setSemaphoreFiles('talk');
            setTimeout(() => {
              tts('Thank you, I am brimming with profound things to share!');
            }, 1000);
          }
        }
        break;
      case 'oldMessage':
        console.log('Old text message:');
        console.log(parsedTextMessage.smsText);
        break;
      case 'connect':
        console.log('Remote web server connected!');
        break;
      case 'welcome':
        console.log("Remote web server says, 'Welcome!'");
        break;
      case 'disconnect':
        console.log('Remote web server disconnected.');
        break;
      default:
        console.log('Unknown result from remote web server:');
        console.log(message);
    }
  }
}

module.exports = RemoteMessageHandler;
if (require.main === module) {
  const SocketServerSubscriber = require('./SocketServerSubscriber');
  const remoteMessageHandler = new RemoteMessageHandler();
  const socketServerSubscriber = new SocketServerSubscriber(
    remoteMessageHandler.handleMessage,
  );
  socketServerSubscriber.start();
}
