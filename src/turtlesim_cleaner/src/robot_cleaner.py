#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist

def move(speed, distance, isFoward):
    velocity_message = Twist()

    if isFoward:
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)

    velocity_message.linear.y = 0
    velocity_message.linear.z = 0
    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    velocity_message.angular.z = 0

    velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    current_distance = 0.0
    # we publish the velocity at 10 Hz (10 times a second)
    loop_rate = rospy.Rate(100)
    t0 = rospy.Time.now()
    
    while not rospy.is_shutdown():
        rospy.loginfo("Turtlesim moves forwards")
        velocity_publisher.publish(velocity_message)

        current_distance = speed * (rospy.Time.now()-t0).to_sec()

        print current_distance
        loop_rate.sleep()

        if not (current_distance < distance):
            rospy.loginfo("reached")
            break
    
    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)


if __name__ == '__main__':
    try:
        rospy.init_node('robot_cleaner_node', anonymous=True)
        velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

        speed = float(raw_input("Enter speed:"))
        distance = float(raw_input("Enter distance:"))
        isFoward = raw_input("Forward: (yes/NO):") 
        isFoward = (isFoward.lower() == "yes" or isFoward.lower() == 'y')

        move(speed, distance, isFoward)
        # spin() simply keeps python from exiting until this node is stopped
        # rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
