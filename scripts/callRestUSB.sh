# This script is used to call the resetUSB.sh script via sudo
# because only root can do this.

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

# Do not creating overlapping runs of this!
if ! (pgrep -f resetUSB.sh>/dev/null)
then
    if (sudo -nl|grep resetUSB > /dev/null)
    then
        sudo -n ${SCRIPTDIR}/resetUSB.sh
    else
        echo "Run setup.sh to add the required SUDO entries!"
        exit 1
    fi
else
    echo "resetUSB already running!"
    exit 1
fi
