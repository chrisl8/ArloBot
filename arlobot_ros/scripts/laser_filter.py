#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan


def callback(data):
    # Option 1) Conform data to specified input/output ranges
    # data.ranges  = [data.range_max if range_val>data.range_max else (data.range_min if range_val<data.range_min else range_val) for range_val in data.ranges]
    # data.ranges  = [data.range_max if range_val>data.range_max else (data.range_max if range_val<data.range_min else range_val) for range_val in data.ranges]
    data.ranges = [
        4.9
        if range_val > data.range_max
        else (4.9 if range_val < data.range_min else range_val)
        for range_val in data.ranges
    ]
    # Option 2) Conform input/output ranges to data
    # IF I set the max to a number, then I have to comment these out,
    # Lest the max always be whatever I set it to above!
    # data.range_max = max(data.range_max,max(data.ranges))
    # data.range_min = min(data.range_min,min(data.ranges))
    pub.publish(data)


# Intializes everything
def start():
    rclpy.init()
    node = rclpy.create_node("laser_filter")
    scan_topic = rospy.get_param("~scan_topic", "xv11")
    global pub
    pub = node.create_publisher(LaserScan, queue_size=10, scan_topic + "_filtered")
    node.create_subscription(LaserScan, scan_topic, callback)
    rospy.spin()


if __name__ == "__main__":
    start()
