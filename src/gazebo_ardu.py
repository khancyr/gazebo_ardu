#!/usr/bin/env python

from __future__ import absolute_import
import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelState
import mavros
from mavros_msgs.msg import State
from mavros_msgs.srv import StreamRate, StreamRateRequest

"""
ROS subscribes to mavros and publish pose data to gazebo model for visualization.
NOT A SIMULATOR, JUST A VISUALIZATION
"""


class GazeboArdu():
    def __init__(self):
        self.model_name = "iris_demo"
        self.is_rover = False
        self.state = False
        self.state_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
        self.sub_pos = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        self.sub_state = rospy.Subscriber("/mavros/state", State, self.state_callback)


        if rospy.has_param("~vehicle_name"):
            self.model_name = rospy.get_param("~vehicle_name")
        if rospy.has_param("~is_rover"):
            self.is_rover = rospy.get_param("~is_rover")
        self.model_state = ModelState()
        self.model_state.model_name = self.model_name

    def run(self):
        mavros.set_namespace(mavros.DEFAULT_NAMESPACE)
        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            rate.sleep()

    def pose_callback(self, position):
        self.model_state.pose = position.pose
        if self.is_rover:
            self.model_state.pose.position.z = 0.1
        #if self.model_state.pose.position.z < 0.1:
        #    self.model_state.pose.position.z = 0.1
        self.state_pub.publish(self.model_state)

    def state_callback(self, state):
        if self.state is not state:
            self.state = state
            set_rate = rospy.ServiceProxy(mavros.get_topic('set_stream_rate'), StreamRate)
            set_rate(stream_id=StreamRateRequest.STREAM_ALL, message_rate=20, on_off=True)

if __name__ == '__main__':
    rospy.init_node("GazeboArdu", anonymous=True)
    gzArdu = GazeboArdu()
    try:
        gzArdu.run()
    except rospy.ROSInterruptException:
        pass

