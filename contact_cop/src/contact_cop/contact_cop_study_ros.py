#!/usr/bin/env python
"""
@package contact_cop
@file contact_cop_study_ros.py
@author Anthony Remazeilles
@brief Analysis of the Center of Pressure related to contact points

Copyright (C) {packageCopyright}
https://www.gnu.org/licenses/gpl.txt
"""

import rospy
from dynamic_reconfigure.server import Server
from contact_cop.cfg import contact_cop_studyConfig

# ROS message & services includes
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import WrenchStamped

# other includes
from contact_cop import contact_cop_study_impl
from copy import deepcopy

# todo set a function to write correctly the name
class roscontact_cop_study(object):
    """
    ROS interface class, handling all communication with ROS
    """
    def __init__(self):
        """
        Attributes definition
        """
        self.component_data_ = contact_cop_study_impl.contact_cop_studyData()
        self.component_config_ = contact_cop_study_impl.contact_cop_studyConfig()
        self.component_implementation_ = contact_cop_study_impl.contact_cop_studyImplementation()

        srv = Server(contact_cop_studyConfig, self.configure_callback)
        self.cop_ = rospy.Publisher('cop', Point, queue_size=1)
        self.marker_cop_ = rospy.Publisher('marker_cop', MarkerArray, queue_size=1)
        self.wrench_ = rospy.Subscriber('wrench', WrenchStamped, self.topic_callback_wrench)

    def topic_callback_wrench(self, msg):
        """
        callback called at message reception
        """
        self.component_data_.in_wrench = msg
        self.component_data_.in_wrench_updated = True

    def configure_callback(self, config, level):
        """
        callback on the change of parameters dynamically adjustable
        """
        self.component_config_.force_th = config.force_th
        return config

    def configure(self):
        """
        function setting the initial configuration of the node
        """
        return self.component_implementation_.configure(self.component_config_)

    def activate_all_output(self):
        """
        activate all defined output
        """
        self.component_data_.out_cop_active = True
        self.component_data_.out_marker_cop_active = True
        pass

    def set_all_output_read(self):
        """
        set related flag to state that input has been read
        """
        self.component_data_.in_wrench_updated = False
        pass

    def update(self, event):
        """
        @brief update function

        @param      self The object
        @param      event The event

        @return { description_of_the_return_value }
        """
        self.activate_all_output()
        config = deepcopy(self.component_config_)
        data = deepcopy(self.component_data_)
        self.set_all_output_read()
        self.component_implementation_.update(data, config)

        try:
            self.component_data_.out_cop_active = data.out_cop_active
            self.component_data_.out_cop = data.out_cop
            if self.component_data_.out_cop_active:
                self.cop_.publish(self.component_data_.out_cop)
            self.component_data_.out_marker_cop_active = data.out_marker_cop_active
            self.component_data_.out_marker_cop = data.out_marker_cop
            if self.component_data_.out_marker_cop_active:
                self.marker_cop_.publish(self.component_data_.out_marker_cop)
        except rospy.ROSException as error:
            rospy.logerr("Exception: {}".format(error))


def main():
    """
    @brief Entry point of the package.
    Instanciate the node interface containing the Developer implementation
    @return nothing
    """
    rospy.init_node("contact_cop_study", anonymous=True)

    node = roscontact_cop_study()
    if not node.configure():
        rospy.logfatal("Could not configure the node")
        rospy.logfatal("Please check configuration parameters")
        rospy.logfatal("{}".format(node.component_config_))
        return

    rospy.Timer(rospy.Duration(1.0 / 100), node.update)
    rospy.spin()
