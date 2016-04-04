for i in $(ls /dev/ttyUSB* 2> /dev/null)
do
udevadm info -a -n $i|grep -m 1 product|grep Propeller|grep QuickStart > /dev/null
if [ $? -eq 0 ]
then
echo $i
fi
done
