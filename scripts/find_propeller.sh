for i in $(ls /dev/ttyUSB*)
do
udevadm info -a -n $i|grep -m 1 product|grep Propeller > /dev/null
if [ $? -eq 0 ]
then
echo $i
fi
done

