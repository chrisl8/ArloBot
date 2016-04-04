#!/bin/bash
for i in $(ls /dev/ttyACM* 2> /dev/null)
    do
    udevadm info -a -n $i|grep -m 1 manufacturer|grep Teensyduino > /dev/null
    if [ $? -eq 0 ]
    then
        echo $i
    fi
done

