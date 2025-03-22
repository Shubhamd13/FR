#!/usr/bin/env python3

from math import sqrt, atan2, pi
import numpy as np
import rospy
import tf
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
import pandas as pd
import os

class Turtlebot():
    def __init__(self):
        # Publishers and rate.
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)
        self.rate = rospy.Rate(10)

        # Reset odometry.
        for _ in range(10):
            self.reset_pub.publish(Empty())
            self.rate.sleep()

        # Set fixed goal [x, y].
        self.goal = np.array([6.0, 0.0])
        
        # Robot state: internal velocity and pose.
        self.vel = np.array([0.0, 0.0])
        self.pose = Pose2D()
        rospy.Subscriber("odom", Odometry, self.odom_callback)

        # Controller parameters.
        self.dt = 0.1      # time step.
        self.ksi =      # relaxation time constant.
        self.robot_radius =   # physical radius + boundary you want to give to avoid collision

        # Obstacle avoidance parameters.
        self.dhor =    # sensing radius. You should tune this parameter
        self.timehor =  # time horizon for collision avoidance. You should tune this parameter

        # set desired speed
        self.dspeed = 

        # Obstacle data: for each cylinder, store position, velocity, and radius.
        self.obstacles = {
            "cylinder1": {"pos": np.array([0.0, 0.0]),
                          "vel": np.array([0.0, 0.0]),
                          "radius": 0.1},
            "cylinder2": {"pos": np.array([0.0, 0.0]),
                          "vel": np.array([0.0, 0.0]),
                          "radius": 0.1},
        }

        # trajectories to for plotting
        self.traj_c1 = []
        self.traj_c2 = []
        self.traj_robot = []

        # Subscribe to obstacle topics.
        rospy.Subscriber("/cylinder1/odometry", Odometry, self.obs1_odom_callback)
        rospy.Subscriber("/cylinder1/radius", Float32, self.obs1_radius_callback)
        rospy.Subscriber("/cylinder2/odometry", Odometry, self.obs2_odom_callback)
        rospy.Subscriber("/cylinder2/radius", Float32, self.obs2_radius_callback)

        self.run()

    def computeForces(self):
        # Compute the goal-directed force.
        return F

    def compute_ttc(self, obs):
        """
        Compute time-to-collision (TTC) between the robot and an obstacle.
        Returns 0 if already colliding, or infinity if no collision is predicted.
        """
        return tau

    def run(self):
        rospy.loginfo("Starting closed-loop control with obstacle avoidance and P-controller...")
        Kp_linear = 1.0   # Linear gain.
        Kp_angular = 2.0  # Angular gain.
        goal_threshold = 0.2  # Stop if within 0.2 meters of goal.

        while not rospy.is_shutdown():
            pos = np.array([self.pose.x, self.pose.y])
            # Stop if close enough to the goal.
            if np.linalg.norm(self.goal - pos) < goal_threshold:
                self.vel_pub.publish(Twist())
                rospy.loginfo("Goal reached. Stopping robot.")
                self.save_trajectories()
                return

            # Compute total force and update internal velocity.
            F = 
            # adjust the velocity

            # Determine desired heading.
            desired_angle = atan2(self.vel[1], self.vel[0])
            angle_error = desired_angle - self.pose.theta
            while angle_error > pi:
                angle_error -= 2 * pi
            while angle_error < -pi:
                angle_error += 2 * pi

            desired_speed = np.linalg.norm(self.vel) #changed this to controller
            twist = Twist()
            twist.linear.x = Kp_linear * desired_speed
            twist.angular.z = Kp_angular * angle_error

            self.vel_pub.publish(twist)
            rospy.loginfo("Twist: linear=%.2f, angular=%.2f", twist.linear.x, twist.angular.z)
            self.rate.sleep()

    def odom_callback(self, msg):
        quat = [msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w]
        (_, _, yaw) = tf.transformations.euler_from_quaternion(quat)
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        self.pose.theta = yaw

        self.traj_robot.append([rospy.Time.now().to_sec(), self.pose.x, self.pose.y])

    def obs1_odom_callback(self, msg):
        self.obstacles["cylinder1"]["pos"] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.obstacles["cylinder1"]["vel"] = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y])

        self.traj_c1.append([rospy.Time.now().to_sec(), msg.pose.pose.position.x, msg.pose.pose.position.y])

    def obs1_radius_callback(self, msg):
        self.obstacles["cylinder1"]["radius"] = msg.data

    def obs2_odom_callback(self, msg):
        self.obstacles["cylinder2"]["pos"] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.obstacles["cylinder2"]["vel"] = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y])

        self.traj_c2.append([rospy.Time.now().to_sec(), msg.pose.pose.position.x, msg.pose.pose.position.y])

    def obs2_radius_callback(self, msg):
        self.obstacles["cylinder2"]["radius"] = msg.data

    def save_trajectories(self):
        print("saving the trajectory")
        traj_robot = np.array(self.traj_robot)
        traj_c1 = np.array(self.traj_c1)
        traj_c2 = np.array(self.traj_c2)

        np.save(os.path.expanduser("~")+'/catkin_ws/src/ee144w25/src/lab7/traj_robot.npy', traj_robot)
        np.save(os.path.expanduser("~")+'/catkin_ws/src/ee144w25/src/lab7/traj_c1.npy', traj_c1)
        np.save(os.path.expanduser("~")+'/catkin_ws/src/ee144w25/src/lab7/traj_c2.npy', traj_c2)


        

if __name__ == '__main__':
    rospy.init_node('time_based')
    try:
        Turtlebot()
    except rospy.ROSInterruptException:
        pass


