#!/usr/bin/env python

import rospy
import PyKDL
import math
import tf2_ros
import time
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class Simulator:
    def __init__(self):
        rospy.init_node('simulator_node', log_level=rospy.INFO)
        self.robotList = rospy.get_param('~robot_list')
        self.dT = float(rospy.get_param('~dT', 0.1))

        self._do_sim = False

        self.cmdVel_sub = dict()
        self.odom_pub = dict()
        self.tf_pub = dict()
        self.cmd_vel = dict()
        self.pose = dict()
        self.crashed = dict()
        self.oldTime = rospy.Time.now()
        self.newTime = rospy.Time.now()

        self.clock_pub = rospy.Publisher("clock", Clock, queue_size=10)
        self.clock_sub = rospy.Subscriber("/clock", Clock, self.simulation_node, queue_size=1)

        for robot in self.robotList:
            self.cmdVel_sub[robot] = rospy.Subscriber(robot + "/cmd_vel",
                                                      Twist, self.cmd_vel_callback, robot, queue_size=1)
            self.odom_pub[robot] = rospy.Publisher(robot + "/odom", Odometry, queue_size=10)
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_pub = tf2_ros.TransformBroadcaster()
            self.tf_list = tf2_ros.TransformListener(self.tf_buffer)
            self.cmd_vel[robot] = Twist()
            self.pose[robot] = rospy.wait_for_message(robot + "/initialpose", Pose)
            rospy.loginfo("Received initial position for " + robot + "\n"+str(self.pose[robot]))
            self.crashed[robot] = False

            self.publish_data(robot)

    def main(self):
        while True:
            msg = Clock()
            msg.clock = msg.clock.from_sec(rospy.get_time() + self.dT)
            self.clock_pub.publish(msg)
            # Without sleeping, usual cycle takes 0.001 seconds
            # time.sleep(0.005) - it will sleep for 0.005 + usual cycle time

    def cmd_vel_callback(self, data, robot):
        self.cmd_vel[robot] = data

    def publish_data(self, robot):
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

        odom_msg = Odometry()
        odom_msg.header.stamp = self.newTime
        odom_msg.header.frame_id = robot + "/odom"
        odom_msg.child_frame_id = robot + "/base_link"
        odom_msg.pose.pose = self.pose[robot]
        odom_msg.twist.twist = self.cmd_vel[robot]
        self.odom_pub[robot].publish(odom_msg)

    def find_crashes(self):
        for robot_1 in self.robotList:
            for robot_2 in self.robotList[self.robotList.index(robot_1)+1:]:
                if self.crashed[robot_1] and self.crashed[robot_2]: continue
                if not robot_1 == robot_2:
                    try:
                        trans_1 = self.tf_buffer.lookup_transform("world", robot_1 + "/base_link", rospy.Time(0))
                        trans_2 = self.tf_buffer.lookup_transform("world", robot_2 + "/base_link", rospy.Time(0))

                        dist = math.sqrt((trans_1.transform.translation.x - trans_2.transform.translation.x) ** 2 +
                                         (trans_1.transform.translation.y - trans_2.transform.translation.y) ** 2)

                        if dist < 0.5:
                            self.crashed[robot_1] = True
                            self.crashed[robot_2] = True
                            rospy.logwarn("%s and %s crashed!", robot_1, robot_2)

                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException):
                        continue

    def simulation_node(self, new_time):
        self.newTime = new_time.clock
        # simulate movement for each individual robot in simulated time
        for robot in self.robotList:
            if not self.crashed[robot]:
                delta_time = (self.newTime - self.oldTime).to_sec()
                w = self.cmd_vel[robot].angular.z
                v = self.cmd_vel[robot].linear.x
                yaw = PyKDL.Rotation.Quaternion(self.pose[robot].orientation.x,
                                                self.pose[robot].orientation.y,
                                                self.pose[robot].orientation.z,
                                                self.pose[robot].orientation.w).GetEulerZYX()[0]
                if w == 0:
                    self.pose[robot].position.x += v * math.cos(yaw) * delta_time
                    self.pose[robot].position.y += v * math.sin(yaw) * delta_time
                else:
                    r = v / w
                    self.pose[robot].position.x += r * (- math.sin(yaw) + math.sin(yaw + w * delta_time))
                    self.pose[robot].position.y += r * (  math.cos(yaw) - math.cos(yaw + w * delta_time))
                    yaw += w* self.dT
                    yawQuaternion = PyKDL.Rotation().RotZ(yaw).GetQuaternion()
                    [self.pose[robot].orientation.x,
                     self.pose[robot].orientation.y,
                     self.pose[robot].orientation.z,
                     self.pose[robot].orientation.w] = yawQuaternion
            self.publish_data(robot)
        self.find_crashes()
        self.oldTime = self.newTime

if __name__ == '__main__':
    sim = Simulator()
    sim.main()
