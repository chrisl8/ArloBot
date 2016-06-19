var personalData = require('./personalData');

function SocketServerSubscriber(messageHandler) {
    this.remoteServer = personalData.cloudServer.service + '://' + personalData.cloudServer.fqdn;
    this.messageHandler = messageHandler;
}

SocketServerSubscriber.prototype.start = function() {
    var self = this;
    var io = require('socket.io-client'),
        socket = io.connect(this.remoteServer, {
            port: personalData.cloudServer.port
        });
        socket.on('connect', function() {
            self.messageHandler({
                event: 'connect'
            });
            socket.emit('new robot', personalData.robotName);
        });
        socket.on('event', function(data) {
            // Use this for testing any incoming event you want to.
            // Create more specific "events" when you know what you want to do.
            messageHandler({
                event: 'event',
                data: data
            })
        });
        socket.on('newMessage', function(data) {
            self.messageHandler({
                event: 'newMessage',
                data: data
            });
        });
        socket.on('oldMessage', function(data) {
            console.log({
                event: 'oldMessage',
                data: data
            });
        });
        socket.on('welcome', function() {
            self.messageHandler({
                event: 'welcome'
            });
        });
        socket.on('disconnect', function() {
            messageHandler({
                event: 'disconnect'
            })
        });
};

module.exports = SocketServerSubscriber;
if (require.main === module) {
    var socketServerSubscriber = new SocketServerSubscriber(console.log);
    socketServerSubscriber.start();
}

