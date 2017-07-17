#!/usr/bin/env python

import rospy
import tf2_ros
import time
import threading
import subprocess, signal, os
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from msg_pkg.msg import Statistics


class ClockNode:
     def __init__(self):
        rospy.init_node('clock_node', log_level=rospy.INFO)
        
        self.robotList = rospy.get_param('robot_list')
        self.time_modifier = float(rospy.get_param('~time_modifier', 3))
        self.clocks_per_second = float(rospy.get_param('~clocks_per_second', 10))
        
        self.frame_duration = 1.0 / self.clocks_per_second
        
        self.clock_pub = rospy.Publisher("clock", Clock, queue_size=10)
 
        self.plan_sub = dict()
        self.goal_sub = dict()
        self.exit_sub = dict()
        self.robot_is_done = dict()
        self.planner_active = dict()
        self.pause_clock = False
        
        
        for robot in self.robotList:
            self.plan_sub[robot] = rospy.Subscriber(robot + "/plan",
                                                      Path, self.plan_callback, robot, queue_size=1)
            self.goal_sub[robot] = rospy.Subscriber(robot + "/goal",
                                                      PoseStamped, self.goal_callback, robot, queue_size=1)
            self.exit_sub[robot] = rospy.Subscriber("exit_performance/" + robot,
                                                      Statistics, self.exit_callback, robot, queue_size=1)
            self.robot_is_done[robot] = False
     
     def exit_callback(self, goal, robot):
        self.robot_is_done[robot] = True
        print robot
        print goal
        if all(flag for flag in self.robot_is_done.values()):
            p = subprocess.Popen(['ps', '-A'], stdout=subprocess.PIPE)
            out, err = p.communicate()
            for line in out.splitlines():
                if 'roslaunch' in line:
                    pid = int(line.split(None, 1)[0])
                    os.kill(pid, signal.SIGINT)
     
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
        if not rospy.is_shutdown():
            threading.Timer(self.frame_duration, self.main).start()
        if not self.pause_clock:
            msg = Clock()
            msg.clock = msg.clock.from_sec(rospy.get_time() + self.frame_duration*self.time_modifier)                        
            self.clock_pub.publish(msg)

if __name__ == '__main__':
    clock_node = ClockNode()
    raw_input('Press enter to start /clock node...\n')
    clock_node.main()
    
