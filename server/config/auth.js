// config/auth.js
const personalData = require('../../node/personalData');

// expose our config directly to our application using module.exports
module.exports = {

    'facebookAuth': {
        'clientID': personalData.cloudServer.faceBook.appID, // your App ID
        'clientSecret': personalData.cloudServer.faceBook.appSecret, // your App Secret
        'callbackURL': personalData.cloudServer.faceBook.callbackURL,
        'passReqToCallback': true,
        'enableProof': true,
        'profileFields': ['id', 'email', 'name'] //This
    },

    'twitterAuth': {
        'consumerKey': 'your-consumer-key-here',
        'consumerSecret': 'your-client-secret-here',
        'callbackURL': 'http://localhost:8080/auth/twitter/callback'
    },

    'googleAuth': {
        'clientID': 'your-secret-clientID-here',
        'clientSecret': 'your-client-secret-here',
        'callbackURL': 'http://localhost:8080/auth/google/callback'
    }

};
