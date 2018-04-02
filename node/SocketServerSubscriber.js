const personalData = require('./personalData');
// eslint-disable-next-line import/no-extraneous-dependencies
const io = require('socket.io-client');

class SocketServerSubscriber {
  constructor(messageHandler) {
    this.remoteServer = `${personalData.cloudServer.service}://${
      personalData.cloudServer.fqdn
    }:${personalData.cloudServer.port}`;
    this.messageHandler = messageHandler;
  }

  start() {
    const socket = io(this.remoteServer);
    socket.on('connect', () => {
      this.messageHandler({
        event: 'connect',
      });
      socket.emit('new robot', personalData.robotName);
    });
    socket.on('event', (data) => {
      // Use this for testing any incoming event you want to.
      // Create more specific "events" when you know what you want to do.
      this.messageHandler({
        event: 'event',
        data,
      });
    });
    socket.on('newMessage', (data) => {
      this.messageHandler({
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
      this.messageHandler({
        event: 'welcome',
      });
    });
    socket.on('disconnect', () => {
      this.messageHandler({
        event: 'disconnect',
      });
    });
  }
}

module.exports = SocketServerSubscriber;

if (require.main === module) {
  const socketServerSubscriber = new SocketServerSubscriber(console.log);
  socketServerSubscriber.start();
}
