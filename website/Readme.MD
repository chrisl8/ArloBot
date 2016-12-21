# NOTICE: These are just my notes for how I got here. You do NOT need to do any of this.
In fact it may break things.

The only thing you may want to do is if you make changes to the site you can compile them by running: 
`npm run tsc`

Everything else here is just so I know how I got here.

# To build this, first follow the quick setup at:
https://angular.io/docs/ts/latest/quickstart.html
* NOTE: Those instructions change with every iteration of Angular 2, so this site does not line up with the
current tutorial anymore.

# Then install: 
`npm install`
* This is done by the quick install script, but if you just cloned this source you need to do this.

# Then add a few more things:
```
npm install ng2-bootstrap --save
npm install socket.io-client --save
npm install bootstrap --save
```
* These are already in the package.json file, so no need to add them again.

# Then run this to get the socket.io-client typings set up:
`./node_modules/.bin/typings install dt~socket.io-client --global --save`
* The result of the above is already in the repo.

# For ROS we also need these:
```
mkdir lib;cd lib
wget http://cdn.robotwebtools.org/roslibjs/current/roslib.js
wget http://cdn.robotwebtools.org/EventEmitter2/0.4.14/eventemitter2.js
```
* I now get these via symlinks to the node backend code.

# Finally run:
`npm start` 
at least once to get it all put together.
* After that you can just update the site with: 
`npm run tsc`

# Based on:
https://github.com/jgainfort/angular2MyApp

* This is actually an attempt to interface an Angular2 web site with my Arlobot. So it is very specialized.
* Maybe I'll fork and rebuild jgainfort's example to work with Angular 2 someday.

