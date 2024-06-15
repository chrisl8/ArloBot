/*
 * Original source by Kevin Kuei from
 * https://github.com/wennycooper/mybot_autodocking
 */

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>


ros::Publisher cmd_vel_pub;
ros::Subscriber irValue_sub;

unsigned int foundDockingStation = 0;
geometry_msgs::msg::Vector3 vec3;
double acquisitionAngularZ;
double approachAngularZ;
double linearX;

void irValueCallback(const geometry_msgs::msg::Vector3 &vector3)
{
  //omega_left = vector3.x;
  //omega_right  = vector3.y;

  
  vec3.x = vector3.x;
  vec3.y = vector3.y;

  if (foundDockingStation) {
    geometry_msgs::msg::TwistPtr cmd(new geometry_msgs::msg::Twist());
    cmd->linear.x = linearX;
    cmd->angular.z = approachAngularZ * (vector3.x - vector3.y);
    if (vector3.x || vector3.y)
      cmd_vel_pub.publish(cmd);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto nh = rclcpp::Node::make_shared("autodocking_irCode");

  nh.param("/autodocking_irCode/acquisition_angular_z", acquisitionAngularZ, 0.4);
  nh.param("/autodocking_irCode/approach_angular_z", approachAngularZ, 0.1);
  nh.param("/autodocking_irCode/linear_x", linearX, -0.08);

  printf("acquisitionAngularZ: %f, approachAngularZ: %f, linearX: %f\n", acquisitionAngularZ, approachAngularZ, linearX);

  cmd_vel_pub = nh.advertise<geometry_msgs::msg::Twist>("/cmd_vel", 50);
  irValue_sub = nh.subscribe("/irCode", 10, irValueCallback);

  vec3.x = 0; //initialized with some big values
  vec3.y = 0;

  rclcpp::Rate r(10);

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node); 

    // Uncomment and rebuild for debugging
    //printf("x=%f, y=%f\n", vec3.x, vec3.y);
    if (!foundDockingStation && vec3.x == 0.0 && vec3.y == 0.0) {
      geometry_msgs::msg::TwistPtr cmd(new geometry_msgs::msg::Twist());
      cmd->angular.z = acquisitionAngularZ; // const rotate until
      cmd->linear.x = 0.0;
      cmd_vel_pub.publish(cmd);
    } else {
      foundDockingStation = 1;
    }
      
    r.sleep();
  }
  return 0;
}
