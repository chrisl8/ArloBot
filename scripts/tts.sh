#!/usr/bin/env node

// This is an example of how to call a node module from the command line,
var tts = require('./tts.js');
tts(process.argv[2]);

