#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseAction
from actionlib_msgs.msg import GoalStatusArray

def callback(data):
    sta = data.status_list
    print('goalstatus.status')

def listener():
    rospy.init_node('goal_status')
    rospy.Subscriber('/move_base/status', GoalStatusArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()