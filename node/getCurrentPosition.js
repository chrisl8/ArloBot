//var webModel = require('./webModel');
const spawn = require('child_process').spawn;

function runGetPositionScript() {
  return new Promise((resolve, reject) => {
    let rawData;
    const process = spawn('unbuffer', ['../scripts/getPositionHelperScript.sh']);
    process.stdout.on('data', (data) => {
      if (data.indexOf('Translation') !== -1) {
        rawData = data.toString();
        process.kill();
      }
    });

    process.on('error', (error) => {
      console.error('Get position error:');
      console.error(error);
      reject(error);
    });

    process.on('close', () => {
      if (rawData) {
        resolve(rawData);
      } else {
        console.log(`Get position failed: ${rawData}`);
        reject('No data');
      }
    });
  });
}

async function getCurrentPosition() {
  const goalTargetPose = {
    position: {
      x: null,
      y: null,
      z: null,
    },
    orientation: {
      x: null,
      y: null,
      z: null,
      w: null,
    },
  };

  let positionData;
  try {
    positionData = await runGetPositionScript();
  } catch (e) {
    console.error('Error getting current position:');
    console.error(e);
    return false;
  }
  positionData.split('\n').forEach((element) => {
      if (element.indexOf('Translation') > -1) {
        //console.log(element);
        const reducedElement = element.replace(/\[|\]|,/g, '');
        const splitEelement = reducedElement.split(' ');
        goalTargetPose.position.x = splitEelement[2];
        goalTargetPose.position.y = splitEelement[3];
        goalTargetPose.position.z = splitEelement[4];
      } else if (element.indexOf('Quaternion') > -1) {
        //console.log(element);
        const reducedElement = element.replace(/\[|\]|,/g, '');
        const splitEelement = reducedElement.split(' ');
        goalTargetPose.orientation.x = splitEelement[4];
        goalTargetPose.orientation.y = splitEelement[5];
        goalTargetPose.orientation.z = splitEelement[6];
        goalTargetPose.orientation.w = splitEelement[7];
      }
    },
  );

  const currentPosition = 'pose: { position: { x: ' + goalTargetPose.position.x + ', y: ' + goalTargetPose.position.y + ', z: ' + goalTargetPose.position.z + ' }, orientation: { x: ' + goalTargetPose.orientation.x + ', y: ' + goalTargetPose.orientation.y + ', z: ' + goalTargetPose.orientation.z + ', w: ' + goalTargetPose.orientation.w + '} }';

  // callback(currentPosition, wayPointName);

  return currentPosition;

}


module.exports = getCurrentPosition;

if (require.main === module) {
  (async () => {
    try {
      const position = await getCurrentPosition();
      console.log(position);
    } catch (e) {
      console.error('Error getting position from ROS:');
      console.error(e);
    }
  })();
}

/*
 If I get the CURRENT position and send it, it should stay still, right?

 chrisl8@ArloBot:~/arlobot$ rosrun tf tf_echo /map /base_link
 At time 1410717899.809
 - Translation: [1.777, 0.951, 0.101]
 - Rotation: in Quaternion [0.000, 0.000, -0.501, 0.865]

 This works IF you set the goal.target_pose.header.frame_id = "map"
 '''
 '''
 goal.target_pose.pose.position.x = 1.777
 goal.target_pose.pose.position.y = 0.951
 goal.target_pose.pose.position.z = 0.101
 goal.target_pose.pose.orientation.x = 0.0
 goal.target_pose.pose.orientation.y = 0.0
 goal.target_pose.pose.orientation.z = -0.501
 goal.target_pose.pose.orientation.w = 0.865
 */

// Testing Send to same position, for use in another module:
/*
 // Add frame_id: 'map' header to position for sending to move_base
 var sendPosition = '{ header: { frame_id: "map" }, ' + currentPosition + ' }';
 console.log(sendPosition);

 var newProcess = spawn('unbuffer', ['rostopic', 'pub', '/move_base_simple/goal', 'geometry_msgs/PoseStamped', sendPosition]);
 newProcess.stdout.setEncoding('utf8');
 newProcess.stdout.on('data', function(data) {
 console.log(data);
 //if (data.indexOf('Translation') > -1) {
 //    rawData = data;
 //    process.kill();
 //}
 });
 */
