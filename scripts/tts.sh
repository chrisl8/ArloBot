#!/usr/bin/env node

// This is an example of how to call a node module from the command line,
var tts = require('../node/tts.js');
tts(process.argv[2]);
