/* Here is the "source" in the Angular 2 TypeScript for socket.io
 * Be sure to install socket.io:
 * npm install socket.io-client --save
 * and add it to your HTML:
 * <script src="node_modules/socket.io-client/socket.io.js"></script>
 * after all of the other <script> lines.
 * 
 * Finally you have to install the "typing" for socket.io-client:
 * ./node_modules/.bin/typings install socket.io-client --ambient --save
 * Otherwise io() will show up as an Unresolved function or method in your IDE.
 * Read more about Typings at https://www.npmjs.com/package/typings
 * 
 * This is where you put the URL for the websocket server,
 * In an easy world this is your web host / like this:
 * let socket = io('/');
 * Otherwise you may need an entire URL like this:
 * let socket = io('http://arlobot:8080/');
 * or just:
 * let socket = io('arlobot:8080');
 */
let socket = io('/');

export class SocketService {
    socket;

    constructor() {
        this.socket = socket;
    }

    emit(type:string) {
        this.socket.emit(type);
    }

    emitData(type:string, data:string) {
        this.socket.emit(type, data);
    }
}