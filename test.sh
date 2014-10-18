if [[ $# -ne 1 ]]
then
echo 'Please provide a map name for saving to later!'
exit
else
echo ${1}
fi
echo ~/rosmaps/${1}