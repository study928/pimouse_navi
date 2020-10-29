#!/usr/bin/env python

import rospy, tf, actionlib, math
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

waypoints = [
    [(1.23259091377,3.46958374977,0.0),(0.0,0.0,0.239503200671,0.970895574647)],
    [(7.41637706757,-1.8959107399,0.0),(0.0,0.0,-0.658820112152,0.752300511647)],
    [(2.03319215775,-5.13414382935,0.0),(0.0,0.0,0.955895699917,-0.293706334422)],
    [(-3.98838114738,3.32823562622,0.0),(0.0,0.0,0.781378418044,0.624057503613)]
]

def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose

if __name__ == '__main__':
    rospy.init_node('patrol')
    listener = tf.TransformListener()

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    listener.waitForTransform('map', 'base_footprint', rospy.Time(), rospy.Duration(4.0))

    while True:
        for pose in waypoints:
            goal = goal_pose(pose)
            client.send_goal(goal)
                
            while True:
                now = rospy.Time.now()
                listener.waitForTransform('map', 'base_footprint', now, rospy.Duration(4.0))
                position, quaternion = listener.lookupTransform('map', 'base_footprint', now)

                if(math.sqrt((position[0]-goal.target_pose.pose.position.x)**2 + (position[1]-goal.target_pose.pose.position.y)**2) <= 0.5):
                    print ("next goal!!")
                    break
                elif goal.target_pose.pose.position.x == -3.98838114738:
                    break
                else:
                    rospy.sleep(0.5)

        if goal.target_pose.pose.position.x == -3.98838114738:
            print('finish!!')
            break