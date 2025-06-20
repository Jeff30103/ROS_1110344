#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
from tf.transformations import euler_from_quaternion

class PatrolBot:
    def __init__(self):
        rospy.init_node('patrol_node')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)

        self.obstacle_threshold = 0.6
        self.obstacle_detected = False
        self.avoiding = False
        self.avoid_start_time = None
        self.avoid_duration = 2.0  # seconds

        self.waypoints = [
            (0.085, 3.239),
            (-3.827, 3.841),
            (-4.132, 1.833),
            (-5.443, -1.132),
            (-0.493, -3.162),
            (-2.529, -5.002),
            (3.556, -4.639),
            (6.036, -1.196),
            (2.336, 1.415)
        ]
        self.current_wp_index = 0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

    def scan_callback(self, scan_msg):
        # 前方±20度的範圍
        front_indices = list(range(340, 360)) + list(range(0, 20))
        dists = [scan_msg.ranges[i] for i in front_indices if not math.isinf(scan_msg.ranges[i])]
        min_dist = min(dists) if dists else float('inf')
        self.obstacle_detected = min_dist < self.obstacle_threshold

    def odom_callback(self, odom_msg):
        self.current_x = odom_msg.pose.pose.position.x
        self.current_y = odom_msg.pose.pose.position.y

        q = odom_msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw

    def shortest_angular_distance(self, from_angle, to_angle):
        delta = to_angle - from_angle
        while delta > math.pi:
            delta -= 2 * math.pi
        while delta < -math.pi:
            delta += 2 * math.pi
        return delta

    def run(self):
        while not rospy.is_shutdown():
            twist = Twist()
            wp_x, wp_y = self.waypoints[self.current_wp_index]
            dx = wp_x - self.current_x
            dy = wp_y - self.current_y
            dist = math.sqrt(dx**2 + dy**2)
            target_yaw = math.atan2(dy, dx)
            angle_diff = self.shortest_angular_distance(self.current_yaw, target_yaw)

            # ===== 避障模式 =====
            if self.obstacle_detected and not self.avoiding:
                rospy.logwarn(" Obstacle detected! Start avoidance.")
                self.avoiding = True
                self.avoid_start_time = time.time()

            if self.avoiding:
                if time.time() - self.avoid_start_time < self.avoid_duration:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.5
                    self.cmd_pub.publish(twist)
                    self.rate.sleep()
                    continue
                else:
                    rospy.loginfo(" Finished avoidance.")
                    self.avoiding = False

            # ===== 到達目標點 =====
            if dist < 0.3:
                rospy.loginfo(f" Reached waypoint {self.current_wp_index + 1}")
                self.current_wp_index = (self.current_wp_index + 1) % len(self.waypoints)
                continue

            # ===== 正常移動：邊走邊修角度 =====
            twist.linear.x = 0.2
            twist.angular.z = 0.8 * angle_diff  # 比例控制角速度
            self.cmd_pub.publish(twist)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        bot = PatrolBot()
        bot.run()
    except rospy.ROSInterruptException:
        pass
