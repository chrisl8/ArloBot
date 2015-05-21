cd ../node
forever start --killSignal=SIGINT --minUptime=5000 --spinSleepTime=10000 --l="/tmp/arloBehavior.log" --o="/tmp/arloBehavior.log" --e="/tmp/arloBehavior.log" arloBehavior.js
forever list
firefox http://localhost:8080/localmenu.html
forever stop arloBehavior.js
rm "/tmp/arloBehavior.log"
