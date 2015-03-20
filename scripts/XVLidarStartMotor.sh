SCRIPTDIR=$(cd $(dirname "$0") && pwd)
XV11PORT=$(${SCRIPTDIR}/find_XVLidar.sh)
stty -F ${XV11PORT} cs8 115200 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts
echo "ResetConfig" > ${XV11PORT}
#echo "MotorOn" > ${XV11PORT}
#echo "" > ${XV11PORT}

