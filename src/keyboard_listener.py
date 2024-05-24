#!/usr/bin/env python3
import rospy
import std_msgs.msg
import keyboard

def keyboard_listener():
    rospy.init_node('keyboard_listener')
    pub = rospy.Publisher('keyboard_input', std_msgs.msg.String, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        if keyboard.is_pressed('w'):
            pub.publish('w')
            rospy.loginfo("Move forward")
        elif keyboard.is_pressed('s'):
            pub.publish('s')
            rospy.loginfo("Move backward")
        elif keyboard.is_pressed('a'):
            pub.publish('a')
            rospy.loginfo("Move left")
        elif keyboard.is_pressed('d'):
            pub.publish('d')
            rospy.loginfo("Move right")
        else:
            pass
        rate.sleep()

if __name__ == "__main__":
    try:
        keyboard_listener()
    except rospy.ROSInterruptException:
        pass