#!/usr/bin/env python
"""
@package contact_analysis
@file contact_cop_impl.py
@author Anthony Remazeilles
@brief Analysis of the Center of Pressure related to contact points

Copyright (C) {packageCopyright}
https://www.gnu.org/licenses/gpl.txt
"""

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import WrenchStamped

# protected region user include package begin #
import numpy
from visualization_msgs.msg import Marker
# protected region user include package end #

class ContactCopConfig(object):
    """
    set of elements accessible through dynamic reconfigure
    autogenerated: don't touch this class
    """
    def __init__(self):
        self.force_th = 1.0
        pass

    def __str__(self):
        msg = "Instance of ContactCopConfig class: {"
        msg += "force_th: {} ".format(self.force_th)
        msg += "}"
        return msg

class ContactCopData(object):
    """
    set of input / output handled through the update methods
    autogenerated: don't touch this class
    """
    def __init__(self):
        """
        Definition of the ContactCopData attributes
        """
        # input data
        self.in_wrench = WrenchStamped()
        self.in_wrench_updated = bool()
        # output data
        self.out_cop = Point()
        self.out_cop_active = bool()
        self.out_marker_cop = MarkerArray()
        self.out_marker_cop_active = bool()
        pass

    def __str__(self):
        msg = "Instance of ContactCopData class: \n {"
        msg += "in_wrench: {} \n".format(self.in_wrench)
        msg += "in_wrench_updated: {} \n".format(self.in_wrench_updated)
        msg += "out_cop: {} \n".format(self.out_cop_active)
        msg += "out_cop_active: {} \n".format(self.out_cop_active)
        msg += "out_marker_cop: {} \n".format(self.out_marker_cop_active)
        msg += "out_marker_cop_active: {} \n".format(self.out_marker_cop_active)
        msg += "}"
        return msg

class ContactCopPassthrough(object):
    """
    set of passthrough elements slightly violating interface / implementation separation
    Autogenerated: don't touch this class
    """
    def __init__(self):
        """ Class to contain variable breaking the interface separation
        """
        pass

class ContactCopImplementation(object):
    """
    Class to contain Developer implementation.
    """
    def __init__(self):
        """
        Definition and initialisation of class attributes
        """
        self.passthrough = ContactCopPassthrough()

        # protected region user member variables begin #
        self.marker = create_marker_point("cop", "cop_frame")
        self.history_length = 10

        cur_id = 2
        cur_a = 1 - 1.0 / self.history_length

        self.markers = list()

        for i in range(self.history_length):
            marker = create_marker_point("cop", "cop_frame")
            marker.color.a = cur_a
            marker.id = cur_id
            self.markers.append(marker)

            cur_a = cur_a - 1.0 / self.history_length
            cur_id += 1

        print "Finished with: {}".format(cur_a)
        # self.marker.header.frame_id = frame_id
        # self.marker.ns = "cop"
        # self.marker.id = 1
        # self.marker.type = Marker.POINTS
        # self.marker.action = Marker.ADD
        # self.marker.color.r = 1.0
        # self.marker.color.a = 1.0
        # self.marker.scale.x = 0.01
        # self.marker.scale.y = 0.01
        # self.marker.scale.z = 0.01
        # self.marker.pose.position.x = 0
        # self.marker.pose.position.y = 0
        # self.marker.pose.position.z = 0
        # self.marker.pose.orientation.x = 0
        # self.marker.pose.orientation.y = 0
        # self.marker.pose.orientation.z = 0
        # self.marker.pose.orientation.w = 1.0
        # self.marker.lifetime = rospy.Duration(1.0)
        # protected region user member variables end #

    def configure(self, config):
        """
        @brief configuration of the implementation
        @param      self The object
        @param      config set of configuration parameters
        @return True on success
        """
        # protected region user configure begin #
        # protected region user configure end #
        return True


    def update(self, data, config):
        """
        @brief { function_description }

        @param      self The object
        @param      data data handled through the ros class
        @param      config parameters handled through dyn. recon.

        @return nothing
        """
        # protected region user update begin #

        # if the force norm is higher than the norm force defined, we just publish it
        wrench = data.in_wrench.wrench
        force = numpy.asarray([wrench.force.x, wrench.force.y, wrench.force.z])
        torque = numpy.asarray([wrench.torque.x, wrench.torque.y, wrench.torque.z])

        force_norm = numpy.linalg.norm(force)

        if force_norm > config.force_th and force[2] != 0:
            data.out_cop.x = - torque[1] / force[2]
            data.out_cop.y = torque[0] / force[2]
            data.out_cop.z = 0.0

            self.marker.points = list()
            self.marker.points.append(data.out_cop)
            self.marker.header.frame_id = data.in_wrench.header.frame_id

            data.out_marker_cop.markers = [self.marker]
        else:
            data.out_cop_active = False
            data.out_marker_cop_active = False

        # protected region user update end #
        pass

    # protected region user additional functions begin #

def create_marker_point(ns='debug', frame_id='map'):
    """
    creation of a rviz marker of type point
    :param ns: namespace to use
    :param frame_id: reference frame to state
    :return:
    """
    point_marker = Marker()
    point_marker.header.frame_id = frame_id
    point_marker.ns = ns
    point_marker.id = 1
    point_marker.type = Marker.POINTS
    point_marker.action = Marker.ADD
    point_marker.color.r = 1.0
    point_marker.color.a = 1.0
    point_marker.scale.x = 0.005
    point_marker.scale.y = 0.005
    point_marker.scale.z = 0.005
    point_marker.lifetime = rospy.Duration(0.1)
    return point_marker
    # protected region user additional functions end #
