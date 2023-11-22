#!/usr/bin/env
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import re

from std_msgs.msg import String

def move_client():
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    path = read_ros_path("../path/lpl.rospath")
    
    for (x, y) in path:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0

        client.send_goal(goal)
        wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


def read_ros_path(filename):
    file = open(filename)
    point_pattern = re.compile("(\d+): (-?\d+\.\d+), (-?\d+\.\d+)")
    path = []
    lines = file.readlines()
    assert lines[0] == "<number>: <map_x>, <map_y>\n"
    for line in lines[1:]:
        (number, x, y) = point_pattern.search(line).groups()
        path.append((float(x), float(y)))
    file.close()
    return path


# def talk_to_me():
#     pub = rospy.Publisher('talking_topic', String, queue_size=10)
#     rospy.init_node('publisher_node', anonymous=True)
#     rate = rospy.Rate(1)
#     rospy.loginfo("Test Publisher Started")
#     while not rospy.is_shutdown():
#         msg = "Hello world - %s" % rospy.get_time()
#         pub.publish(msg)
#         rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        result = move_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
