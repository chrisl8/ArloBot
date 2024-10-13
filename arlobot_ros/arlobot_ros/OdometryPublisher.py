import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdometryPublisher(Node):
    def __init__(self):
        self.odomTestX = 0.0
        print("Initializing Odometry Publisher")
        super().__init__('odometry_publisher')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        print("Odometry Publisher initialized")
        self.timer_ = self.create_timer(1.0/5.0, self.publish_odometry)

    def publish_odometry(self):
        print("publish_odometry")
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.odomTestX = self.odomTestX + 0.01
        msg.pose.pose.position.x = self.odomTestX
        print(self.odomTestX)
        # msg.pose.pose.position.x = x
        # msg.pose.pose.position.y = y
        # msg.pose.pose.position.z = z
        # msg.pose.pose.orientation.x = quat_x
        # msg.pose.pose.orientation.y = quat_y
        # msg.pose.pose.orientation.z = quat_z
        # msg.pose.pose.orientation.w = quat_w
        self.publisher_.publish(msg)

def main(args=None):
    print("starting OdometryPublisher")
    rclpy.init(args=args)
    print("1")
    node = OdometryPublisher()
    print("2")
    rclpy.spin(node)
    print("3")
    node.destroy_node()
    print("4")
    rclpy.shutdown()
    print("5")

if __name__ == '__main__':
    main()