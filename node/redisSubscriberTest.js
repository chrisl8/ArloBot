const redis = require('redis');

const redisServer = 'pi2';

const homeEvents = redis.createClient(6379, redisServer, {});
// If you want to subscribe on Redis,
// and also get things,
// you must have two clients, because a subscribed client
// cannot issue any commands once it is subscribed.
const getRedisMessages = redis.createClient(6379, redisServer, {});

// What if the redis server doesn't exist?
// var failedRedis = redis.createClient(6379, 'pi', {});
// Be sure to have an on.('error' handler!
// Note that it will "back off and retry" doubling the time
// with each retry.
// By default there is no upper limit to the retry delay.
// You can change that if you like.
// I don't know if this will explode and launch a missile when the retry
// time reaches infinity? I assume not. :P
// failedRedis.on('error', function(err) {
//    console.log('failedRedis connection failed: ' + err);
// });
// And just to be safe for our "production" channel too.
homeEvents.on('error', (err) => {
  console.log(`homeEvents Redis connection failed: ${err}`);
});
// Really, you have to do it for EVERY connection you set up!
getRedisMessages.on('error', (err) => {
  console.log(`homeEvents Redis connection failed: ${err}`);
});
// This could be important if your app only uses Redis "if" it is available,
// and doesn't require it as a part of its basic function.

const redisChannel = 'householdevents';
homeEvents.on('message', (channel, message) => {
  console.log(`Channel: ${channel} says: ${message}`);
  if (message === 'aDoorOpened' || message === 'aDoorClosed') {
    // You can publish any message, but if the message === 'aDoorOpened':
    // publish householdevents aDoorOpened
    // then it will get the members of the set 'opendoors'
    // Just be sure to populate the new set member BEFORE publishing
    // the message!
    // NOTE that messages are case sensitive/preserving in Redis
    getRedisMessages.smembers('opendoors', (err, res) => {
      if (err) {
        console.log(`Error getting opendoors list: ${err}`);
      } else {
        console.log(`Open Doors: ${res}`);
      }
    });
    // What if we also grab something we know does not exist?
    /*
    getRedisMessages.smembers('doesnotexist', function(err, res) {
        if (err) {
            console.log('Error getting doesnotexist list: ' + err);
        } else {
            console.log('doesnotexist: ' + res);
        }
    });
    */
    // A non-existent set looks just like an empty set.
    // HOWEVER if 'doesnotexist' is not a set, you WILL get an error:
    // Error getting doesnotexist list: Error: WRONGTYPE Operation against a key holding the wrong kind of value
  }
});

homeEvents.subscribe(redisChannel);
