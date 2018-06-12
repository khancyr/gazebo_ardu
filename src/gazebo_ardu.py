#!/usr/bin/env python

from __future__ import absolute_import
import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelState
import mavros
from mavros_msgs.msg import State
from mavros_msgs.srv import StreamRate, StreamRateRequest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

"""
ROS subscribes to mavros and publish pose data to gazebo model for visualization.
NOT A SIMULATOR, JUST A VISUALIZATION
"""
vehicle_type = ["Quadrotor", "Ground rover"]

class GazeboArdu():
    def __init__(self):
        self.model_name = "iris_demo"
        self.type = "Quadrotor"
        self.state = False
        self.state_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
        self.sub_pos = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        self.sub_state = rospy.Subscriber("/mavros/state", State, self.state_callback)
        self.vehicle_type_sub = None


        if rospy.has_param("~vehicle_name"):
            self.model_name = rospy.get_param("~vehicle_name")
        self.model_state = ModelState()
        self.model_state.model_name = self.model_name

    def run(self):
        mavros.set_namespace(mavros.DEFAULT_NAMESPACE)
        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            rate.sleep()

    def pose_callback(self, position):
        self.model_state.pose = position.pose
        if self.type == "Ground rover":
            self.model_state.pose.position.z = 0.1
        #if self.model_state.pose.position.z < 0.1:
        #    self.model_state.pose.position.z = 0.1
        self.state_pub.publish(self.model_state)

    def state_callback(self, state):
        if self.state is not state.connected:
            self.state = state.connected
            set_rate = rospy.ServiceProxy(mavros.get_topic('set_stream_rate'), StreamRate)
            set_rate(stream_id=StreamRateRequest.STREAM_ALL, message_rate=20, on_off=True)
            self.vehicle_type_sub = rospy.Subscriber("/diagnostics", DiagnosticArray, self.type_callback)


    def type_callback(self, diag):
        index1 = [diag.status.index(x) for x in diag.status if x.name == "mavros: Heartbeat"]
        index2 = [diag.status[index1[0]].values.index(x) for x in diag.status[index1[0]].values if x.key == "Vehicle type"]
        self.type = diag.status[index1[0]].values[index2[0]].value
        if self.type in vehicle_type:
            self.vehicle_type_sub.unregister()



if __name__ == '__main__':
    rospy.init_node("GazeboArdu", anonymous=True)
    gzArdu = GazeboArdu()
    try:
        gzArdu.run()
    except rospy.ROSInterruptException:
        pass

