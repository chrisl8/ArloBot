# Use this to stop the motor on the XV11
# I find it annoying to have it spinning 24x7 all of the time
# when ROS isn't even running.
# Plus I worry that it might wear out?

# FYI: This is called by kill_ros.sh, so you do not need to call this manually.

# Grab and save the path to this script
# http://stackoverflow.com/a/246128
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
SCRIPTDIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
# echo ${SCRIPTDIR} # For debugging

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
