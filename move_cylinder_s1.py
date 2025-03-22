#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class MoveCylinder:
    def __init__(self):
        rospy.init_node("collision_avoidance")
        self.c1_pub = rospy.Publisher("/cylinder1/cmd_vel", Twist, queue_size=10)
        self.vel1 = Twist()
        self.vel1.linear.x = -0.2

        self.c2_pub = rospy.Publisher("/cylinder2/cmd_vel", Twist, queue_size=10)
        self.vel2 = Twist()
        self.vel2.linear.x = -0.25

        self.c1_radi_pub = rospy.Publisher("/cylinder1/radius", Float32, queue_size=10)
        self.c2_radi_pub = rospy.Publisher("/cylinder2/radius", Float32, queue_size=10)

        self.rate = rospy.Rate(10)

        self.run()


    def run(self):
        while not rospy.is_shutdown():
            self.c1_pub.publish(self.vel1)
            self.c2_pub.publish(self.vel2)
            self.c1_radi_pub.publish(0.1)
            self.c2_radi_pub.publish(0.1)
            self.rate.sleep()



if __name__ == "__main__":
    try:
        mc = MoveCylinder()
    except rospy.ROSInterruptException:
        pass



