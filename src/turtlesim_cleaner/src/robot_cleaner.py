#!/usr/bin/env python2

import rospy
from math import pi, atan2, sqrt
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

turtlesim_pose = {
    "x": 0,
    "y": 0,
    "theta": 0
}

def poseCallback(pose_message):
    turtlesim_pose["x"] = pose_message.x
    turtlesim_pose["y"] = pose_message.y
    turtlesim_pose["theta"] = pose_message.theta

def degrees2Radians(angle):
    return (angle * pi) / 180.0

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

def rotate(angular_speed, relative_angle, isClockwise):
    velocity_message = Twist()

    if isClockwise:
        velocity_message.angular.z = abs(angular_speed)
    else:
        velocity_message.angular.z = -abs(angular_speed)

    velocity_message.linear.x = 0
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0
    velocity_message.angular.x = 0
    velocity_message.angular.y = 0

    loop_rate = rospy.Rate(50)
    t0 = rospy.Time.now()
    current_angle = .0

    while not rospy.is_shutdown():
        rospy.loginfo("Turtlesim rotate")
        velocity_publisher.publish(velocity_message)

        current_angle = angular_speed * (rospy.Time.now()-t0).to_sec()

        print current_angle
        loop_rate.sleep()

        if not (current_angle < relative_angle):
            rospy.loginfo("reached")
            break
    
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)

def setDesiredOrientation(desiredAngleRadians):
    relativeAngleRadians = desiredAngleRadians - turtlesim_pose["theta"]
    isClockwise = relativeAngleRadians > 0
    rotate(1, abs(relativeAngleRadians), isClockwise)

def getDistance(x1, y1, x2, y2):
    return sqrt((x1-x2)**2 + (y1-y2)**2)

def moveToGoal(goal_pose, distance_tolerance):
    velocity_message = Twist()
    loop_rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        rospy.loginfo("Turtlesim rotate")
        velocity_message.linear.x = .3*getDistance(turtlesim_pose["x"], turtlesim_pose["y"], goal_pose.x, goal_pose.y)
        velocity_message.linear.y = 0
        velocity_message.linear.z = 0
        velocity_message.angular.x = 0
        velocity_message.angular.y = 0
        velocity_message.angular.z = 4*(atan2(goal_pose.y-turtlesim_pose["y"], goal_pose.x-turtlesim_pose["x"]) - turtlesim_pose["theta"])
        
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()

        print(turtlesim_pose["x"], turtlesim_pose["y"], goal_pose.x, goal_pose.y)
        print(getDistance(turtlesim_pose["x"], turtlesim_pose["y"], goal_pose.x, goal_pose.y))
        if getDistance(turtlesim_pose["x"], turtlesim_pose["y"], goal_pose.x, goal_pose.y) < distance_tolerance:
            rospy.loginfo("reached")
            break

        velocity_message.linear.x = 0
        velocity_message.angular.z = 0
        velocity_publisher.publish(velocity_message)

    

if __name__ == '__main__':
    try:
        rospy.init_node('robot_cleaner_node', anonymous=True)
        velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/turtle1/pose", Pose, poseCallback)

        # speed = float(raw_input("Enter speed:"))
        # distance = float(raw_input("Enter distance:"))
        # isFoward = raw_input("Forward: (yes/NO):") 
        # isFoward = (isFoward.lower() == "yes" or isFoward.lower() == 'y')
        # move(speed, distance, isFoward)

        # angular_speed = float(raw_input("Enter angular velocity(degree/sec): "))
        # angle = degrees2Radians(float(raw_input("Enter desire angle (degrees):")))
        # isClockwise = raw_input("Clockwise (yes/NO):")
        # isClockwise = isClockwise.lower() == "yes" or isClockwise.lower() == "y"
        # rotate(angular_speed, angle, isClockwise)

        loop_rate = rospy.Rate(.5)
        # setDesiredOrientation(degrees2Radians(180))
        # loop_rate.sleep()
        # setDesiredOrientation(degrees2Radians(90))
        # loop_rate.sleep()
        # setDesiredOrientation(degrees2Radians(0))
        
        goal_pose = Pose()
        goal_pose.x = 1
        goal_pose.y = 1
        goal_pose.theta = 0
        moveToGoal(goal_pose, 1)
        loop_rate.sleep()

        # spin() simply keeps python from exiting until this node is stopped
        # rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
