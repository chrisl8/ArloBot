# meld config file is here: ~/.gconf/apps/meld/%gconf.xml
echo "Here are all of the functional files that make up the arlobot ROS packages:"
find ~/arlobot/src/ -type f|grep -v package.xml|grep -v CMakeLists.txt
echo "Here are the file names without path:"
find ~/arlobot/src/ -type f|grep -v package.xml|grep -v CMakeLists.txt|awk -F'/' '{ print $NF }'
echo "Now we will run meld on each file that exists in the TurtleBot source to compare them:"
# NOTE: I find this works best if you turn on "ignore blank lines" and add this filter to meld:
#(arlo)|(turtle)bot</stringvalue>
# and I turn on ignoring of comments
# The result is some files are "identical" so then you know they are just there to accomodate renaming,
# and are functionally equivalent to the Turtlebot original.
for i in $(find ~/arlobot/src/ -type f|grep -v package.xml|grep -v CMakeLists.txt|grep -v .stl|grep -v .pyc)
do
if [ -f ~/turtlebot/$(echo $i|sed -e 's\arlo\turtle\g') ]
then
meld ~/turtlebot/$(echo $i|sed -e 's\arlo\turtle\g') $i
fi
done
