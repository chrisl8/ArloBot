#!/usr/bin/env bash
# Grab and save the path to this script
# http://stackoverflow.com/a/246128
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
SCRIPTDIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
# echo ${SCRIPTDIR} # For debugging

# meld config file is here: ~/.gconf/apps/meld/%gconf.xml

echo "Here are all of the functional files that make up the arlobot ROS navigation:"
#find ${SCRIPTDIR}/../src/arlobot_apps/arlobot_navigation/ -type f|grep -v package.xml|grep -v CMakeLists.txt
#echo "Here are the file names without path:"
find ${SCRIPTDIR}/../src/arlobot_apps/arlobot_navigation/ -type f | grep -v package.xml | grep -v CMakeLists.txt | awk -F'/' '{ print $NF }'
echo "Now we will run meld on each file that exists in the TurtleBot source to compare them:"
# NOTE: I find this works best if you turn on "ignore blank lines" and add this filter to meld:
#(arlo)|(turtle)bot</stringvalue>
# and I turn on ignoring of comments
# The result is some files are "identical" so then you know they are just there to accomodate renaming,
# and are functionally equivalent to the Turtlebot original.
roscd turtlebot_navigation
TURTLEBOT_NAVIGATION_FOLDER = $(pwd)
cd
for i in $(find ${SCRIPTDIR}/../src/arlobot_apps/arlobot_navigation/ -type f | grep -v package.xml | grep -v CMakeLists.txt | grep -v .stl | grep -v .pyc); do
  if [[ -f ~/turtlebot/$(echo $i | sed -e 's\arlo\turtle\g') ]]; then
    meld ~/turtlebot/$(echo $i | sed -e 's\arlo\turtle\g') $i
  fi
done

#echo "Here are all of the functional files that make up the arlobot ROS packages:"
#find ${SCRIPTDIR}/../src/ -type f|grep -v package.xml|grep -v CMakeLists.txt
#echo "Here are the file names without path:"
#find ${SCRIPTDIR}/../src/ -type f|grep -v package.xml|grep -v CMakeLists.txt|awk -F'/' '{ print $NF }'
#echo "Now we will run meld on each file that exists in the TurtleBot source to compare them:"
## NOTE: I find this works best if you turn on "ignore blank lines" and add this filter to meld:
##(arlo)|(turtle)bot</stringvalue>
## and I turn on ignoring of comments
## The result is some files are "identical" so then you know they are just there to accomodate renaming,
## and are functionally equivalent to the Turtlebot original.
#for i in $(find ${SCRIPTDIR}/../src/ -type f|grep -v package.xml|grep -v CMakeLists.txt|grep -v .stl|grep -v .pyc)
#do
#if [[ -f ~/turtlebot/$(echo $i|sed -e 's\arlo\turtle\g') ]]; then
#meld ~/turtlebot/$(echo $i|sed -e 's\arlo\turtle\g') $i
#fi
#done
