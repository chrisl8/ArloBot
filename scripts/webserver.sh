#!/usr/bin/env node

// This is an example of how to call a node module from the command line,
var webserver = require('../node/webserver/webserver.js');
webserver.start();
