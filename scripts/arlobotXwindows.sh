cd ../node
forever start --killSignal=SIGINT arloBehavior.js
forever list
firefox http://localhost:8080/localmenu.html
forever stop arloBehavior.js

