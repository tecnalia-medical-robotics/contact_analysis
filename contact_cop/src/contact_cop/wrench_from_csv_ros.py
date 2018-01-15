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
from copy import deepcopy

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
        return self.component_implementation_.configure(self.component_config_)

    def activate_all_output(self):
        """
        activate all defined output
        """
        self.component_data_.out_wrench_active = True
        pass

    def set_all_output_read(self):
        """
        set related flag to state that input has been read
        """
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
            self.component_data_.out_wrench_active = data.out_wrench_active
            self.component_data_.out_wrench = data.out_wrench
            if self.component_data_.out_wrench_active:
                self.wrench_.publish(self.component_data_.out_wrench)
        except rospy.ROSException as error:
            rospy.logerr("Exception: {}".format(error))


def main():
    """
    @brief Entry point of the package.
    Instanciate the node interface containing the Developer implementation
    @return nothing
    """
    rospy.init_node("wrench_from_csv", anonymous=True)

    node = roswrench_from_csv()
    if not node.configure():
        rospy.logfatal("Could not configure the node")
        rospy.logfatal("Please check configuration parameters")
        rospy.logfatal("{}".format(node.component_config_))
        return

    rospy.Timer(rospy.Duration(1.0 / 1000), node.update)
    rospy.spin()
