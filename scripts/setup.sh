# This script is meant to help set up your machine
# and user environment for Metatron and Arlobot to work

SCRIPTDIR=$(cd $(dirname "$0") && pwd)

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

# Install apps needed by Metatron/Arlobot
if ! (which festival>/dev/null)
then
    sudo apt-get install festival
fi

