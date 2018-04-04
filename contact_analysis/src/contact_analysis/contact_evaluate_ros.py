#!/usr/bin/env python
"""
@package contact_analysis
@file contact_evaluate_ros.py
@author Anthony Remazeilles
@brief Analysis of the Center of Pressure related to contact points

Copyright (C) {packageCopyright}
https://www.gnu.org/licenses/gpl.txt
"""

import rospy
import actionlib
from dynamic_reconfigure.server import Server
from contact_analysis.cfg import contact_evaluateConfig

# ROS message & services includes
from geometry_msgs.msg import Point
from contact_msgs.srv import SetString
from contact_msgs.srv import SetString
from contact_msgs.msg import LearnContactAction
from contact_msgs.msg import EvaluateContactAction

# other includes
from contact_analysis import contact_evaluate_impl
from copy import deepcopy

# todo set a function to write correctly the name
class ContactEvaluateROS(object):
    """
    ROS interface class, handling all communication with ROS
    """
    def __init__(self):
        """
        Attributes definition
        """
        self.component_data_ = contact_evaluate_impl.ContactEvaluateData()
        self.component_config_ = contact_evaluate_impl.ContactEvaluateConfig()
        self.component_implementation_ = contact_evaluate_impl.ContactEvaluateImplementation()

        srv = Server(contact_evaluateConfig, self.configure_callback)
        self.cop_ = rospy.Subscriber('cop', Point, self.topic_callback_cop)
        # to enable service name adjustment when loading the node
        remap = rospy.get_param("~load_remap", "load")
        self.load_ = rospy.Service(remap, SetString, self.component_implementation_.callback_load)
        # to enable service name adjustment when loading the node
        remap = rospy.get_param("~store_remap", "store")
        self.store_ = rospy.Service(remap, SetString, self.component_implementation_.callback_store)
        # to enable action name adjustment when loading the node
        remap = rospy.get_param("~learn_remap", "learn")
        self.component_implementation_.passthrough.as_learn = actionlib.SimpleActionServer(remap,
                                                                                                LearnContactAction,
                                                                                                execute_cb=self.component_implementation_.callback_learn,
                                                                                                auto_start=False)
        self.component_implementation_.passthrough.as_learn.start()
        # to enable action name adjustment when loading the node
        remap = rospy.get_param("~evaluate_remap", "evaluate")
        self.component_implementation_.passthrough.as_evaluate = actionlib.SimpleActionServer(remap,
                                                                                                EvaluateContactAction,
                                                                                                execute_cb=self.component_implementation_.callback_evaluate,
                                                                                                auto_start=False)
        self.component_implementation_.passthrough.as_evaluate.start()

    def topic_callback_cop(self, msg):
        """
        callback called at message reception
        """
        self.component_data_.in_cop = msg
        self.component_data_.in_cop_updated = True

    def configure_callback(self, config, level):
        """
        callback on the change of parameters dynamically adjustable
        """
        self.component_config_.frequency = config.frequency
        self.component_config_.obs_duration = config.obs_duration
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
        pass

    def set_all_input_read(self):
        """
        set related flag to state that input has been read
        """
        self.component_data_.in_cop_updated = False
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
        self.set_all_input_read()
        self.component_implementation_.update(data, config)



def main():
    """
    @brief Entry point of the package.
    Instanciate the node interface containing the Developer implementation
    @return nothing
    """
    rospy.init_node("contact_evaluate", anonymous=True)

    node = ContactEvaluateROS()
    if not node.configure():
        rospy.logfatal("Could not configure the node")
        rospy.logfatal("Please check configuration parameters")
        rospy.logfatal("{}".format(node.component_config_))
        return

    rospy.Timer(rospy.Duration(1.0 / 1000), node.update)
    rospy.spin()
