#!/usr/bin/env node

// This is an example of how to call a node module from the command line,
var getMapList = require('../node/getMapList.js');
var mapDir = process.env.HOME + '/.arlobot/rosmaps/';
var formatOutput = function(err, data) {
    data.forEach(function(value) {
        console.log(value.replace('.yaml', ''));
    });
};
getMapList(mapDir, formatOutput);
