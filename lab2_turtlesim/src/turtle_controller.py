#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String 
from geometry_msgs.msg import Twist, Vector3 


def controller():

    # Create an instance of the rospy.Publisher object which we can  use to
    # publish messages to a topic. This publisher publishes messages of type
    # std_msgs/String to the topic /chatter_talk
    turtle_name = sys.argv[1]
    com = rospy.Publisher('%s/cmd_vel' % (turtle_name), Twist, queue_size=10)
    valid_keys= ['w','a','s','d']
    # Loop until the node is killed with Ctrl-C
    while not rospy.is_shutdown():
        # Construct a string that we want to publish (in Python, the "%"
        # operator functions similarly to sprintf in C or MATLAB)
        keys = raw_input("Please use WASD to control the turtle: "). lower()
        while keys not in valid_keys:
                keys = raw_input("Not a valid keys, use WASD: ")
        # Publish our string to the 'turtle1/cmd_vel' topic
        lin= [0,0,0]
        ang=[0,0,0]
        if keys =='w':
            lin[0]= 2.0
        elif keys == 'a':
            ang[2]= 2.0
        elif keys =='s':
            lin[0] = -2.0
        elif keys =='d':
            ang[2]=-2.0

        lin = Vector3(lin[0],lin[1],lin[2])
        ang = Vector3(ang[0], ang[1], ang[2])
        velocity = Twist(lin,ang)
        com.publish(velocity)
        print(velocity)

# This is Python's syntax for a main() method, which is run by default when
# exectued in the shell
if __name__ == '__main__':

    # Run this program as a new node in the ROS computation graph called /talker.
    rospy.init_node('controller', anonymous=True)

    # Check if the node has received a signal to shut down. If not, run the
    # talker method.
    try:
        controller()
    except rospy.ROSInterruptException: pass