# This script is meant to help set up your machine
# and user environment for Metatron and Arlobot to work

SCRIPTDIR=$(cd $(dirname "$0") && pwd)

# Check for node.js installation
if ! (which node>/dev/null)
then
    echo "You do not have node.js installed."
    echo "Please run the following commands to install node.js,"
    echo "or use instructions from http://nodejs.org/ if you prefer,"
    echo "and then run this setup script again."
    echo ""
    echo "cd"
    echo "wget http://nodejs.org/dist/v0.12.0/node-v0.12.0.tar.gz"
    echo "tar xzvf node-v0.12.0.tar.gz"
    echo "cd node-v0.12.0"
    echo "./configure"
    echo "make"
    echo "sudo make install"
fi

# We will use ~/.arlobot to store "private" data
# That is data that doesn't need to be part of
# the public github repo like user tokens,
# sounds, and room maps
if [ ! -d ${HOME}/.arlobot ]
then
    mkdir ${HOME}/.arlobot
fi

ARLOHOME=${HOME}/.arlobot

for i in `ls ${SCRIPTDIR}/dotarlobot/`
do
    if [ -e  ${ARLOHOME}/${i} ]
    then
        git diff ${SCRIPTDIR}/dotarlobot/${i} ${ARLOHOME}/${i}
    fi
    cp -i ${SCRIPTDIR}/dotarlobot/${i} ${ARLOHOME}/
done

if [ ! -d ${ARLOHOME}/rosmaps ]
then
    mkdir ${ARLOHOME}/rosmaps
fi

if [ ! -d ${ARLOHOME}/sounds ]
then
    mkdir ${ARLOHOME}/sounds
fi

if [ ! -d ${ARLOHOME}/status ]
then
    mkdir ${ARLOHOME}/status
fi
chmod -R 777 ${ARLOHOME}/status

# Install apps needed by Metatron/Arlobot
if ! (which festival>/dev/null)
then
    echo "Installing Festival for robot speech"
    echo "You will be asked for your password"
    echo "In order to run apt-get install."
    sudo apt-get install festival
fi

if ! (dpkg -l festvox-en1>/dev/null)
then
    echo "Installing Festival en1 voice for robot speech"
    echo "You will be asked for your password"
    echo "In order to run apt-get install."
    sudo apt-get install festvox-en1
fi

if ! (grep mbrola /etc/festival.scm>/dev/null)
then
    echo "Updating default Festival voice"
    echo "You will be asked for your password"
    echo "To allow updates to /etc/festival.scm"
    sudo ${SCRIPTDIR}/updateFestivalDefaults.sh
fi

