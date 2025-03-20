#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import tf

class WaypointFollower:
    def __init__(self):
        rospy.init_node('waypoint_follower', anonymous=True)

        # Publisher for velocity commands
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber for odometry
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # List of waypoints (x, y) coordinates.
        self.waypoints = rospy.get_param("~waypoints", [(3.0, -2.0), (3.1000000000000014, -1.25), (3.8000000000000007, -1.3499999999999996), (4.5, -1.0999999999999996), (5.200000000000001, -1.25), (5.75, -1.6999999999999993), (5.15, -2.1999999999999993), (5.4, -2.8499999999999996), (5.700000000000001, -3.5), (4.950000000000001, -3.6999999999999993), (4.800000000000001, -4.1)])
        self.current_waypoint_index = 0

        # Tolerance to consider the waypoint reached (meters)
        self.tolerance = rospy.get_param("~tolerance", 0.2)
        # Tolerance for acceptable angular error (radians)
        self.angular_tolerance = rospy.get_param("~angular_tolerance", 0.1)

        # Proportional and derivative gains for angular control
        self.kp_angular = rospy.get_param("~kp_angular", 1.0)
        self.kd_angular = rospy.get_param("~kd_angular", 0.1)
        # Proportional gain for linear control
        self.kp_linear = rospy.get_param("~kp_linear", 0.5)

        # Current robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # For derivative calculation
        self.prev_error_angle = 0.0
        self.prev_time = rospy.Time.now()

        self.rate = rospy.Rate(10)  # 10 Hz

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = tf.transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_yaw = yaw

    def run(self):
        while not rospy.is_shutdown() and self.current_waypoint_index < len(self.waypoints):
            target = self.waypoints[self.current_waypoint_index]
            target_x, target_y = target[0], target[1]

            # Calculate distance and desired angle
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = math.sqrt(dx**2 + dy**2)
            target_angle = math.atan2(dy, dx)

            # Compute angular error and normalize it to [-pi, pi]
            error_angle = target_angle - self.current_yaw
            error_angle = math.atan2(math.sin(error_angle), math.cos(error_angle))

            # Calculate time delta for derivative term
            current_time = rospy.Time.now()
            dt = (current_time - self.prev_time).to_sec()
            derivative = (error_angle - self.prev_error_angle) / dt if dt > 0 else 0.0

            # PD control for angular velocity
            angular_z = self.kp_angular * error_angle + self.kd_angular * derivative

            # Update for next iteration
            self.prev_error_angle = error_angle
            self.prev_time = current_time

            twist_msg = Twist()

            # Move forward only if angular error is within tolerance
            if abs(error_angle) < self.angular_tolerance:
                twist_msg.linear.x = self.kp_linear * distance
                # Limit maximum linear speed
                twist_msg.linear.x = max(min(twist_msg.linear.x, 0.2), -0.2)
            else:
                twist_msg.linear.x = 0.0

            # Limit angular velocity as needed
            twist_msg.angular.z = max(min(angular_z, 0.2), -0.2)

            # Check if waypoint is reached
            if distance < self.tolerance:
                rospy.loginfo("Reached waypoint {}: ({}, {})".format(self.current_waypoint_index, target_x, target_y))
                self.current_waypoint_index += 1
                # Stop briefly at waypoint
                self.cmd_pub.publish(Twist())
                rospy.sleep(1.0)
                continue

            self.cmd_pub.publish(twist_msg)
            self.rate.sleep()

        rospy.loginfo("All waypoints reached!")
        self.cmd_pub.publish(Twist())

if __name__ == '__main__':
    try:
        follower = WaypointFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass
