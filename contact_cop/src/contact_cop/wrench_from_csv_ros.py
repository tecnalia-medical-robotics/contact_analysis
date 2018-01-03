#!/usr/bin/env python
"""
@package contact_cop
@file wrench_from_csv_ros.py
@author Anthony Remazeilles
@brief Analysis of the Center of Pressure related to contact points

Copyright (C) {packageCopyright}
https://www.gnu.org/licenses/gpl.txt
"""

import rospy
from dynamic_reconfigure.server import Server
from contact_cop.cfg import wrench_from_csvConfig

# ROS message & services includes
from geometry_msgs.msg import WrenchStamped

# other includes
from contact_cop import wrench_from_csv_impl

# todo set a function to write correctly the name
class roswrench_from_csv(object):
    """
    ROS interface class, handling all communication with ROS
    """
    def __init__(self):
        """
        Attributes definition
        """
        self.component_data_ = wrench_from_csv_impl.wrench_from_csvData()
        self.component_config_ = wrench_from_csv_impl.wrench_from_csvConfig()
        self.component_implementation_ = wrench_from_csv_impl.wrench_from_csvImplementation()

        srv = Server(wrench_from_csvConfig, self.configure_callback)
        self.wrench_ = rospy.Publisher('wrench', WrenchStamped, queue_size=1)

    def configure_callback(self, config, level):
        """
        callback on the change of parameters dynamically adjustable
        """
        self.component_config_.csv_file = config.csv_file
        return config

    def configure(self):
        """
        function setting the initial configuration of the node
        """
        self.component_implementation_.configure(self.component_config_)

    # todo: this may need to be handled as well
    def activate_all_output(self):
        """
        activate all defined output
        """
        self.component_data_.out_wrench_active = True

    def update(self, event):
        """
        @brief update function

        @param      self The object
        @param      event The event

        @return { description_of_the_return_value }
        """
        self.activate_all_output()

        self.component_implementation_.update(self.component_data_, self.component_config_)

        if self.component_data_.out_wrench_active:
            self.wrench_.publish(self.component_data_.out_wrench)

def main():
    """
    @brief Entry point of the package.
    Instanciate the node interface containing the Developer implementation
    @return nothing
    """
    rospy.init_node("wrench_from_csv", anonymous=True)

    node = roswrench_from_csv()
    node.configure()

    rospy.Timer(rospy.Duration(1.0 / 1000), node.update)
    rospy.spin()