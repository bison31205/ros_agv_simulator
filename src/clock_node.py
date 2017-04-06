#!/usr/bin/env python

import rospy
import tf2_ros
import time
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class ClockNode:
     def __init__(self):
        rospy.init_node('clock_node', log_level=rospy.INFO)
        self.robotList = rospy.get_param('robot_list')
        self.dT = float(rospy.get_param('~dT', 0.1))
        self.clock_pub = rospy.Publisher("clock", Clock, queue_size=10)
        
        self.plan_sub = dict()
        self.goal_sub = dict()
        self.planner_active = dict()
        
        self.pause_clock = False
        
        for robot in self.robotList:
            self.plan_sub[robot] = rospy.Subscriber(robot + "/plan",
                                                      Path, self.plan_callback, robot, queue_size=1)
            self.goal_sub[robot] = rospy.Subscriber(robot + "/goal",
                                                      PoseStamped, self.goal_callback, robot, queue_size=1)
     
     def goal_callback(self, goal, robot):
        self.pause_clock = True
        self.planner_active[robot] = True
        
     def plan_callback(self, plan, robot):
        self.planner_active[robot] = False
        reactivate_clock = True
        for key, value in self.planner_active.iteritems():
            if value == True:
                reactivate_clock = False
                break
        self.pause_clock = not reactivate_clock
        
     def main(self):
         while not rospy.is_shutdown():
            if not self.pause_clock:
                msg = Clock()
                msg.clock = msg.clock.from_sec(rospy.get_time() + self.dT)                        
                self.clock_pub.publish(msg)

if __name__ == '__main__':
    clock_node = ClockNode()
    clock_node.main()
