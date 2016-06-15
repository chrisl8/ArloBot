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

function RemoteMessageHandler() {
}

RemoteMessageHandler.prototype.handleMessage = function (message) {
    /* Sample response:
     Handler got:
     Robot-0 { event: 'newMessage',
     Robot-0   data: '{"smsText":"Test","smsTo":"+13162851661","smsFrom":"+13162087309"}' }
     */

    // Here we make use of some common input:
    if (message.data) {
        const smsData = JSON.parse(message.data);
        if (smsData && smsData.smsText && smsData.smsFrom && smsData.smsFrom === personalData.twilio.my_phone_number) {
            const smsText = smsData.smsText;
            const smsTextArray = smsText.split(' ');
            const smsTextArrayLowerCase = smsTextArray.map(function(value) {
                return value.toLowerCase();
            });
            console.log(`SMS Message was: ${smsText}`);

            // These actions only apply to real time messages, not stored messages.
            if (message.event === 'newMessage') {
                if (smsTextArray[0].toLowerCase() === 'say') {
                    const textToSay = smsText.substr(smsText.indexOf(" ") + 1);
                    tts(textToSay);
                } else if (smsTextArray[0].toLowerCase() === 'goodnight' || (smsTextArray[0].toLocaleLowerCase() === 'good' && smsTextArray[1].toLowerCase() === 'night')) {
                    tts('Good night master.');
                    robotModel.master.isAsleep = true;
                } else if (smsTextArray[0].toLowerCase() === 'goodmorning' || (smsTextArray[0].toLocaleLowerCase() === 'good' && smsTextArray[1].toLowerCase() === 'morning')) {
                    tts('Top of the morning to you master!');
                    robotModel.master.isAsleep = false;
                } else if (smsTextArrayLowerCase.indexOf('quiet') > -1) {
                    tts('Shutting up now.');
                    handleSemaphoreFiles.setSemaphoreFiles('beQuiet');
                } else if (smsTextArrayLowerCase.indexOf('talk') > -1) {
                    handleSemaphoreFiles.setSemaphoreFiles('talk');
                    setTimeout(function() {
                        tts('Thank you, I am brimming with profound things to share!');
                    }, 1000);
                }
            }

        }
    }

};
module.exports = RemoteMessageHandler;
if (require.main === module) {
    const SocketServerSubscriber = require('./SocketServerSubscriber');
    var remoteMessageHandler = new RemoteMessageHandler();
    var socketServerSubscriber = new SocketServerSubscriber(remoteMessageHandler.handleMessage);
    socketServerSubscriber.start();
}
