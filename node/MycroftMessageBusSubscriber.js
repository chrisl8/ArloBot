// TODO: I don't think this code is used.
const io = require('socket.io-client');

function SocketServerSubscriber(messageHandler) {
  // this.remoteServer = personalData.cloudServer.service + '://' + personalData.cloudServer.fqdn;
  this.remoteServer = 'ws://127.0.0.1:8000/events/ws';
  this.messageHandler = messageHandler;
}

SocketServerSubscriber.prototype.start = function() {
  const self = this;
  const socket = io.connect(this.remoteServer, {
    port: 8000,
  });
  socket.on('connect', () => {
    self.messageHandler({
      event: 'connect',
    });
    // socket.emit('new robot', personalData.robotName);
    socket.emit('speak', { metadata: { utterance: 'Hello world' } });
  });
  socket.on('event', (data) => {
    // Use this for testing any incoming event you want to.
    // Create more specific "events" when you know what you want to do.
    messageHandler({
      event: 'event',
      data,
    });
  });
  socket.on('newMessage', (data) => {
    self.messageHandler({
      event: 'newMessage',
      data,
    });
  });
  socket.on('oldMessage', (data) => {
    console.log({
      event: 'oldMessage',
      data,
    });
  });
  socket.on('welcome', () => {
    self.messageHandler({
      event: 'welcome',
    });
  });
  socket.on('disconnect', () => {
    messageHandler({
      event: 'disconnect',
    });
  });
};

module.exports = SocketServerSubscriber;
if (require.main === module) {
  const socketServerSubscriber = new SocketServerSubscriber(console.log);
  socketServerSubscriber.start();
}
