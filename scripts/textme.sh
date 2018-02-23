#!/usr/bin/env node

// This is an example of how to call a node module from the command line,
var textme = require('../node/textme.js');
textme(process.argv[2], true);

