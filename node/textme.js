const personalData = require('./personalData');

function textMe(message, debug) {
    // Send myself text messages with twilio
    // https://www.twilio.com/docs/quickstart/node/programmable-sms

    if (personalData.twilio.account_sid !== "" && personalData.twilio.auth_token !== "") {
        const client = require('twilio')(personalData.twilio.account_sid, personalData.twilio.auth_token);
        // Pass in parameters to the REST API using an object literal notation. The
        // REST client will handle authentication and response serialzation for you.
        client.messages.create({
            to: personalData.twilio.my_phone_number,
            from: personalData.twilio.number,
            body: message
        }, function (error, message) {
            // The HTTP request to Twilio will run asynchronously. This callback
            // function will be called when a response is received from Twilio
            // The "error" variable will contain error information, if any.
            // If the request was successful, this value will be "falsy"
            if (!error) {
                // The second argument to the callback will contain the information
                // sent back by Twilio for the request. In this case, it is the
                // information about the text message you just sent:
                if (debug) {
                    console.log('Success! The SID for this SMS message is:');
                    console.log(message.sid);
                    console.log('Message sent on:');
                    console.log(message.dateCreated);
                }
            } else {
                if (debug) {
                    console.error('Twilio Error:');
                    console.error(error);
                } else {
                    console.error('Twilio Error');
                }
            }
        });
    }
}

module.exports = textMe;

if (require.main === module) {
    if (process.argv.length < 3) {
        console.log("You must provide text for the message, like this:");
        console.log(`node textme.js "test message"`);
        process.exit();
    }
    textMe(process.argv[2], true);
}