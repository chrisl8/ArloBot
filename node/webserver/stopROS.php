<!DOCTYPE html>
<html lang="en">
<head>
<title>Stopping ROS . . .</title>
<meta charset="utf-8">
</head>
<body>
<?php
if (file_exists('arloweb.ini')) {
$ini_array = parse_ini_file("arloweb.ini");
#print_r($ini_array);
#echo("{$ini_array['scriptFolder']}<br/>");
#echo("{$ini_array['rosUser']}<br/>");*/
shell_exec("nohup {$ini_array['scriptFolder']}/stubStopper.sh {$ini_array['scriptFolder']} {$ini_array['rosUser']} &");
echo 'Stopping ROS';
} else {
echo 'ERROR: arloweb.ini not found . . . ';
echo '<script>window.location.href = "./missingConfig.html";</script>';
}
?>
</body>
</html>

