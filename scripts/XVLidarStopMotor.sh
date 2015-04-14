SCRIPTDIR=$(cd $(dirname "$0") && pwd)
XV11PORT=$(${SCRIPTDIR}/find_XVLidar.sh)
stty -F ${XV11PORT} cs8 115200 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts
echo "MotorOff" > ${XV11PORT}
# Experience tells me one time is not enough.
sleep 1
stty -F ${XV11PORT} cs8 115200 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts
echo "MotorOff" > ${XV11PORT}
sleep 1
stty -F ${XV11PORT} cs8 115200 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts
echo "MotorOff" > ${XV11PORT}
#echo "MotorOn" > ${XV11PORT}
#echo "" > ${XV11PORT}

