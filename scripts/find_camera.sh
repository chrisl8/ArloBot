# Find the camera listed on the command line,
# based on output of 'fswebam'
for i in $(ls /dev/video*)
do
fswebcam --verbose --device=${i} 2>&1|grep cap.card|grep ${1} > /dev/null
if [ $? -eq 0 ]
then
echo $i
fi
done

