To build this, first follow the quick setup at:
https://angular.io/docs/ts/latest/quickstart.html

Then install:
npm install

Then add a few more things:
npm install ng2-bootstrap --save
npm install socket.io-client --save
npm install bootstrap --save

Then run this to get the socket.io-client typings set up:
./node_modules/.bin/typings install socket.io-client --ambient --save

Based on:
https://github.com/jgainfort/angular2MyApp

This is actually an attempt to interface an Angular2 web site with my Arlobot. So it is very specialized.

Maybe I'll fork and rebuilt jgainfort's example to work with a Angular now.
