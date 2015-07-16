# Find the camera listed on the command line,
# based on output of 'fswebam'
if [ $# -eq 0 ]
then
echo "You must provide a camera name such as,"
echo "HP or C615"
echo "on the command line."
exit
fi
FOUND=1
for i in $(ls /dev/video*)
do
fswebcam --verbose --device=${i} 2>&1|grep cap.card|grep ${1} > /dev/null
if [ $? -eq 0 ]
then
echo $i
FOUND=0
fi
done
exit ${FOUND}
