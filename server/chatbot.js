var Pandorabot = require('pb-node');
var personalData = require('../node/personalData');

var options = {
    url: 'https://aiaas.pandorabots.com',
    app_id: personalData.cloudServer.pandorabots.app_id,
    user_key: personalData.cloudServer.pandorabots.user_key,
    botname: personalData.cloudServer.pandorabots.botname
};

var bot = new Pandorabot(options);
module.exports = bot;
