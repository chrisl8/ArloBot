// This will handle messages from the remote cloud based web server.
// Presumably these will be mostly SMS messages from Twilio,
// but anything we could want to poll or push to the robot,
// via an Internet accesible web server will arrive here.


function RemoteMessageHandler() {
}

RemoteMessageHandler.prototype.handleMessage = function(message) {
    console.log('Handler got:');
    console.log(message);
};
module.exports = RemoteMessageHandler;
if (require.main === module) {
    var SocketServerSubscriber = require('./SocketServerSubscriber');
    var remoteMessageHandler = new RemoteMessageHandler();
    var socketServerSubscriber = new SocketServerSubscriber(remoteMessageHandler.handleMessage);
    socketServerSubscriber.start();
}

