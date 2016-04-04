for i in $(ls /dev/ttyACM* 2> /dev/null)
do
udevadm info -a -n $i|grep -m 1 product|grep "Numato Lab 1 Channel USB Powered Relay Module" > /dev/null
if [ $? -eq 0 ]
then
echo $i
fi
done

