#!/usr/bin/env python

import rospy
import PyKDL
import math
import tf2_ros
import time
import threading
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path


class Simulator:
    exit_simulator = threading.Event()
    
    def __init__(self):
        self.init_done = False
        rospy.init_node('simulator_node', log_level=rospy.INFO)
        self.robotList = rospy.get_param('robot_list')
        self.showRealTime = rospy.get_param('~show_dt', False)

        self.path_sub = dict()
        self.cmd_vel_sub = dict()
        self.odom_pub = dict()
        self.traj_pub = dict()
        self.tf_pub = dict()
        self.marker_pub = dict()
        self.marker_color = dict()
        self.path = dict()
        self.path_index = dict()
        self.pose = dict()
        self.cmd_vel = dict()
        self.crashed = dict()
        self.oldTime = rospy.Time.now()
        self.newTime = rospy.Time.now()
        
        self.old_real_time = time.clock()

        self.clock_sub = rospy.Subscriber("/clock", Clock, self.simulation_node, queue_size=1)

        for robot in self.robotList:
            self.path_sub[robot] = rospy.Subscriber(robot + "/follow_path",
                                                      Path, self.plan_callback, robot, queue_size=1)
            self.cmd_vel_sub[robot] = rospy.Subscriber(robot + "/cmd_vel",
                                                      Twist, self.cmd_vel_callback, robot, queue_size=1)
            self.odom_pub[robot] = rospy.Publisher(robot + "/odom", Odometry, queue_size=10)
            self.traj_pub[robot] = rospy.Publisher(robot + "/active_path", Path, queue_size=10)
            self.tf_pub = tf2_ros.TransformBroadcaster()
            self.marker_pub[robot] = rospy.Publisher(robot + "/marker", Marker, queue_size=10)
            self.marker_color[robot] = ColorRGBA()
            self.marker_color[robot].a = 1.0
            self.marker_color[robot].r = math.fabs(math.fmod(hash(robot) / 1.4e10, 1))
            self.marker_color[robot].g = math.fabs(math.fmod(hash(robot) / 4.2e10, 1))
            self.marker_color[robot].b = math.fabs(math.fmod(hash(robot) / 6.9e10, 1))
            self.path[robot] = []
            self.path_index[robot] = 0
            self.pose[robot] = rospy.wait_for_message(robot + "/initialpose", Pose)
            #rospy.loginfo("Received initial position for " + robot + "\n"+str(self.pose[robot]))
            self.crashed[robot] = False
            self.cmd_vel[robot] = Twist()

            self.publish_data(robot)
            
        rospy.on_shutdown(self.publish_exit_event)
        self.init_done = True
        
    def publish_exit_event():
        exit_simulator.set()
            
    def publish_active_path(self, robot):
        pub_active_path = Path()
        pub_active_path.header.frame_id = 'world'
        for path in self.path[robot]:
            pub_active_path.poses += path.poses
        
        self.traj_pub[robot].publish(pub_active_path)
            
    def plan_callback(self, data, robot):
        self.path[robot].append(data)
        self.publish_active_path(robot)
    
    def cmd_vel_callback(self, data, robot):
        self.cmd_vel[robot] = data
    
    def print_real_time(self):
        new_real_time = time.clock()
        print '{0}\r'.format(new_real_time-self.old_real_time),
        #print("Progress {:2.1%}".format(x / 10), end="\r")
        self.old_real_time = new_real_time

    def publish_data(self, robot):
        # Publish TF data
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.newTime
        tf_msg.header.frame_id = robot + "/odom"
        tf_msg.child_frame_id = robot + "/base_link"
        tf_msg.transform.translation.x = self.pose[robot].position.x
        tf_msg.transform.translation.y = self.pose[robot].position.y
        tf_msg.transform.translation.z = self.pose[robot].position.z
        tf_msg.transform.rotation.x = self.pose[robot].orientation.x
        tf_msg.transform.rotation.y = self.pose[robot].orientation.y
        tf_msg.transform.rotation.z = self.pose[robot].orientation.z
        tf_msg.transform.rotation.w = self.pose[robot].orientation.w
        self.tf_pub.sendTransform(tf_msg)

        # Publish Odometry data
        odom_msg = Odometry()
        odom_msg.header.stamp = self.newTime
        odom_msg.header.frame_id = robot + "/odom"
        odom_msg.child_frame_id = robot + "/base_link"
        odom_msg.pose.pose = self.pose[robot]
        odom_msg.twist.twist = self.cmd_vel[robot]
        self.odom_pub[robot].publish(odom_msg)
        
        # Publish RViz markers data
        
        marker_msg = Marker()
        marker_msg.header.stamp = self.newTime
        marker_msg.header.frame_id = "world"
        marker_msg.ns = "marker"
        marker_msg.type = 3
        marker_msg.action = 0
        marker_msg.pose = self.pose[robot]
        marker_msg.scale.x = 0.5
        marker_msg.scale.y = 0.5
        marker_msg.scale.z = 0.25
        marker_msg.color = self.marker_color[robot]
        self.marker_pub[robot].publish(marker_msg)
        
        
    def find_crashes(self):
        for robot_1 in self.robotList:
            for robot_2 in self.robotList[self.robotList.index(robot_1)+1:]:
                if self.crashed[robot_1] and self.crashed[robot_2]: continue
                if not robot_1 == robot_2:
                    dist = math.sqrt((self.pose[robot_1].position.x - self.pose[robot_2].position.x) ** 2 +
                                     (self.pose[robot_1].position.y - self.pose[robot_2].position.y) ** 2)

                    if dist < 0.5:
                        self.crashed[robot_1] = True
                        self.crashed[robot_2] = True
                        rospy.logwarn("%s and %s crashed!", robot_1, robot_2)
                        # rospy.loginfo(str(self.pose[robot_1].position))
                        # rospy.loginfo(str(self.pose[robot_2].position))
                        

    def calc_new_pose(self, robot, time):
        # If empty path, return current pose
        if len(self.path[robot]) == 0:
            return self.pose[robot]
        # If cmd_vel is zero, robot is stationary
        if self.cmd_vel[robot].linear.x == 0:
            return self.pose[robot]
        
        # Calculate next position on Path
        spent_time = 0
        pose_old = self.pose[robot]
        while True:
            if self.path_index[robot] + 1 == len(self.path[robot][0].poses):
                self.path[robot].pop(0)
                self.path_index[robot] = 0
                self.publish_active_path(robot)
                if len(self.path[robot]) == 0:
                    self.cmd_vel[robot] = Twist()
                break
                
            pose_new = self.path[robot][0].poses[self.path_index[robot]+1].pose
            
            dist = math.sqrt((pose_old.position.x - pose_new.position.x) ** 2 +
                             (pose_old.position.y - pose_new.position.y) ** 2)
            
            next_pose_time = dist / self.cmd_vel[robot].linear.x
            if not spent_time + next_pose_time > time:
                spent_time += next_pose_time
                self.path_index[robot] += 1
                pose_old = pose_new
            else:
                left_time = time - spent_time
                percent = left_time / next_pose_time
                
                dist_x = (pose_new.position.x - pose_old.position.x) * percent
                dist_y = (pose_new.position.y - pose_old.position.y) * percent
                
                pose_old.position.x += dist_x
                pose_old.position.y += dist_y
                
                break    
            
        return pose_old

    def simulation_node(self, new_time):
        self.newTime = new_time.clock
        if self.showRealTime:
            self.print_real_time()
        if not self.init_done:
            return
        # simulate movement for each individual robot in simulated time
        for robot in self.robotList:
            if not self.crashed[robot]:
                delta_time = (self.newTime - self.oldTime).to_sec()
                self.pose[robot] = self.calc_new_pose(robot, delta_time)
            self.publish_data(robot)
        self.find_crashes()
        self.oldTime = self.newTime

if __name__ == '__main__':
    sim = Simulator()
    sim.exit_simulator.wait()
    
