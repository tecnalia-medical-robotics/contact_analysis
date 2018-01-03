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

# other includes
from contact_cop import contact_cop_study_impl

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

    def topic_callback_wrench(self, msg):
        """
        callback called at message reception
        """
        self.component_data_.in_wrench = msg

    def configure(self):
        """
        function setting the initial configuration of the node
        """
        return self.component_implementation_.configure(self.component_config_)

    # todo: this may need to be handled as well
    def activate_all_output(self):
        """
        activate all defined output
        """

    def update(self, event):
        """
        @brief update function

        @param      self The object
        @param      event The event

        @return { description_of_the_return_value }
        """
        self.activate_all_output()

        self.component_implementation_.update(self.component_data_, self.component_config_)

        try:
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

    rospy.Timer(rospy.Duration(1.0 / 1000), node.update)
    rospy.spin()
