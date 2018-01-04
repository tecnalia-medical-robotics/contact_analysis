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

# ROS message & services includes
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool
from plot_tool.srv import PlotPose

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

        self.wrench_ = rospy.Subscriber('wrench', WrenchStamped, self.topic_callback_wrench)
        self.loop_ = rospy.Subscriber('loop', Bool, self.topic_callback_loop)
        # to enable service name adjustment when loading the node
        remap = rospy.get_param("~display_remap", "display")
        self.component_implementation_.passthrough.client_display = rospy.ServiceProxy(remap, PlotPose);

    def topic_callback_wrench(self, msg):
        """
        callback called at message reception
        """
        self.component_data_.in_wrench = msg
        self.component_data_.in_wrench_updated = True

    def topic_callback_loop(self, msg):
        """
        callback called at message reception
        """
        self.component_data_.in_loop = msg
        self.component_data_.in_loop_updated = True

    def configure(self):
        """
        function setting the initial configuration of the node
        """
        return self.component_implementation_.configure(self.component_config_)

    def activate_all_output(self):
        """
        activate all defined output
        """
        pass

    def set_all_output_read(self):
        """
        set related flag to state that input has been read
        """
        self.component_data_.in_wrench_updated = False
        self.component_data_.in_loop_updated = False
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

    rospy.Timer(rospy.Duration(1.0 / 50), node.update)
    rospy.spin()
