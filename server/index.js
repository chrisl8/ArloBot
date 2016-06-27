var personalData = require('../node/personalData');
var redis = require('redis'),
    client = redis.createClient();
var twilio = require('twilio');
// Redis
var redis = require('redis');
var redisServer = 'localhost';
var arloBot = redis.createClient(6379, redisServer, {});
// If you want to subscribe on Redis,
// and also get things,
// you must have two clients, because a subscribed client
// cannot issue any commands once it is subscribed.
var getRedisMessages = redis.createClient(6379, redisServer, {});

// What if the redis server doesn't exist?
//var failedRedis = redis.createClient(6379, 'pi', {});
// Be sure to have an on.('error' handler!
// Note that it will "back off and retry" doubling the time
// with each retry.
// By default there is no upper limit to the retry delay.
// You can change that if you like.
// I don't know if this will explode and launch a missile when the retry
// time reaches infinity? I assume not. :P
//failedRedis.on('error', function(err) {
//    console.log('failedRedis connection failed: ' + err);
//});
// And just to be safe for our "production" channel too.
arloBot.on('error', function (err) {
    console.log('arloBot Redis connection failed: ' + err);
});
// Really, you have to do it for EVERY connection you set up!
getRedisMessages.on('error', function (err) {
    console.log('arloBot Redis connection failed: ' + err);
});
// This could be important if your app only uses Redis "if" it is available,
// and doesn't require it as a part of its basic function.

var express = require('express');
var cookieParser = require('cookie-parser');
var session = require('express-session');
var RedisStore = require('connect-redis')(session); // Express.js says NOT to use their session store for production.

// Fancy Express Web Server
// All of my "static" web pages are in the public folder
var app = express();
app.disable('x-powered-by'); // Do not volunteer system info!

app.use(session({
    store: new RedisStore({
        host: 'localhost',
        prefix: 'robot-site-sessions'
    }),
    secret: '0sRqRRX3XIyhssWPNBj2LWDjfmmIgEYuk8zXkTWrYpL4c38vSdNf32whnEsFuoDR',
    saveUninitialized: false, // True for built in, false for redis-connect
    resave: false
}));
app.use(cookieParser());

app.set('views', __dirname + '/views');
app.set('view engine', 'ejs');

app.use(express.static(__dirname + '/public'));

// For parsing Post data
var bodyParser = require('body-parser');
app.use(bodyParser.json()); // to support JSON-encoded bodies
app.use(bodyParser.urlencoded({ // to support URL-encoded bodies
    extended: true
}));

var robotSubscribers = [];
var Robot = require('./Robot');
var port = process.env.PORT || 3003;
var webServer = app.listen(port);
// with Socket.io!
var socket = require('socket.io').listen(webServer);

function sendOldMessages() {
    if (robotSubscribers.length > 0) {
        client.lpop('twilio', function(listName, item) {
            if (item !== null) {
            console.log(item);
            socket.sockets.emit('oldMessage', item);
            sendOldMessages();
        }});
    }
}

socket.sockets.on('connection', onSocketConnection);

function onSocketConnection(client) {
    console.log('Socket connection started:');
    //console.log(client);
    
    client.on('new robot', onNewRobot);
    client.on('disconnect', onClientDisconnect);
}

function onNewRobot(data) {
    var newRobot = new Robot(this.id, data);
    robotSubscribers.push(newRobot);
    socket.sockets.emit('welcome');
    console.log(this.id, data);
    console.log(robotSubscribers);
    sendOldMessages();
}

function onClientDisconnect() {
    console.log('Robot has disconnected: ' + this.id);

    var robotToRemove = robotById(this.id);

    if (!robotToRemove) {
        console.log('Robot not found.');
        return;
    }

    robotSubscribers.splice(robotSubscribers.indexOf(robotToRemove), 1);

    console.log(robotSubscribers);
}

function robotById(id) {
    var i;
    for (i=0; i < robotSubscribers.length; i++) {
        if (robotSubscribers[i].id === id) {
            return robotSubscribers[i];
        }
    }
    return false;
}

app.use(express.static(__dirname + '/public'));

// Public Robot Front Page
app.get('/', function (req, res) {
    res.render('landingPage');
});

// Redirect to local robot URL
app.get('/redirect', function (req, res) {
    var clientResponse = res;
    // Default, hoping you named your computer 'arlobot',
    // and that the name can be resolved on your network.
    var robotURL = 'http://arlobot:8080/index2.html';
    getRedisMessages.get('robotURL', function (err, res) {
        if (err) {
            console.log('Error getting robotURL: ' + err);
        } else if (res === null) {
            console.log('robotURL not set.');
        } else {
            robotURL = res;
            console.log('robotURL: ' + res);
        }
        clientResponse.redirect(robotURL);
    });
});

// This allows the robot to tell the server in the cloud what his local URL is,
// Then you can use a public URL, even one written on the robot, for anyone
// to find the robot, even on a strange network where you do not know what IP it has.
// TO test with curl: (Set the URL as desired and the server name as desired.
// curl -v -H "Accept: application/json" -H "Content-type: application/json" --data '{"localURL": "http://192.168.7.115:8080/index2.html", "password": "sueprSecret1785"}' http://localhost:3003/updateRobotURL
app.post('/updateRobotURL', function (req, res) {
    var password = 'sueprSecret1785';
    if (personalData.cloudServer.password && personalData.cloudServer.password.length > 0) {
        password = personalData.cloudServer.password;
    }
    var urlOK = req.body.localURL && req.body.localURL.length > 0;
    // TODO: Use real authentication and SSL if we are ever afraid of this being hijacked.
    var passwordOK = req.body.password && req.body.password === password;
    if (urlOK && passwordOK) {
        getRedisMessages.set('robotURL', req.body.localURL, function (err, reply) {
            if (err) {
                res.sendStatus(500);
                console.log('Error setting robotURL: ' + err);
            } else {
                res.sendStatus(200);
                console.log(reply);

            }
        });
        console.log(req.body.localURL);
    } else {
        if (!passwordOK) {
            res.sendStatus(403);
            console.log('Bad password');
        } else {
            res.sendStatus(400);
            console.log('URL not set.');
        }
    }
});

app.post('/twilio', function(request, response) {
    if (twilio.validateExpressRequest(request, personalData.twilio.auth_token, {url: personalData.twilio.smsWebhook})) {
        var messageForRedis = {
            smsText: request.body.Body,
            smsTo: request.body.To,
            smsFrom: request.body.From
        };
        console.log(messageForRedis.smsFrom, messageForRedis.smsText);
        messageForRedis = JSON.stringify(messageForRedis);
        // Tell Twilio we got the message, and reply to the sender
        response.header('Content-Type', 'text/xml');
        if (robotSubscribers.length > 0) {
            socket.sockets.emit('newMessage', messageForRedis);
            response.send('<Response><Sms>Got it!</Sms></Response>');
        } else {
            // Save the message in REDIS
            client.rpush("twilio", messageForRedis);
            response.send('<Response><Sms>Sorry, nobody is home, try again later.</Sms></Response>');
        }
    } else {
        console.log('Invalid. Does not appear to be from Twilio!');
        response.sendStatus(403);
    }
});

