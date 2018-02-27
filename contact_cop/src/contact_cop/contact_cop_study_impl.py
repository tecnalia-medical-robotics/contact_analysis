#!/usr/bin/env python
"""
@package contact_cop
@file contact_cop_study_impl.py
@author Anthony Remazeilles
@brief Analysis of the Center of Pressure related to contact points

Copyright (C) {packageCopyright}
https://www.gnu.org/licenses/gpl.txt
"""

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool
from plot_tool.srv import PlotPose, PlotPoseRequest
# todo do not need both of them

# protected region user include files begin #
import numpy
from visualization_msgs.msg import Marker
# protected region user include files end #

class contact_cop_studyConfig(object):
    """
    set of elements accessible through dynamic reconfigure
    """
    def __init__(self):
        self.force_th = 1.0
        pass

    def __str__(self):
        msg = "Instance of contact_cop_studyConfig class: {"
        msg += "force_th: {} ".format(self.force_th)
        msg += "}"
        return msg

class contact_cop_studyData(object):
    """
    set of input / output handled through the update methods
    """
    def __init__(self):
        """
        Definition of the contact_cop_studyData attributes
        """
        # autogenerated: don't touch this class
        # input data
        self.in_wrench = WrenchStamped()
        self.in_wrench_updated = bool()
        self.in_loop = Bool()
        self.in_loop_updated = bool()

        # output data
        self.out_cop = Point()
        self.out_cop_active = bool()
        self.out_marker_cop = MarkerArray()
        self.out_marker_cop_active = bool()

        pass

    def __str__(self):
        msg = "Instance of contact_cop_studyData class: \n {"
        msg += "in_wrench: {} \n".format(self.in_wrench)
        msg += "in_wrench_updated: {} \n".format(self.in_wrench_updated)
        msg += "in_loop: {} \n".format(self.in_loop)
        msg += "in_loop_updated: {} \n".format(self.in_loop_updated)
        msg += "out_cop: {} \n".format(self.out_cop_active)
        msg += "out_cop_active: {} \n".format(self.out_cop_active)
        msg += "out_marker_cop: {} \n".format(self.out_marker_cop_active)
        msg += "out_marker_cop_active: {} \n".format(self.out_marker_cop_active)
        msg += "}"
        return msg

class contact_cop_studyPassthrough(object):
    """
    set of passthrough elements slightly violating interface / implementation separation
    """
    def __init__(self):
        """ Class to contain variable breaking the interface separation
        """
        self.client_display = None
        pass

class contact_cop_studyImplementation(object):
    """
    Class to contain Developer implementation.
    """
    def __init__(self):
        """
        Definition and initialisation of class attributes
        """
        self.passthrough = contact_cop_studyPassthrough()

        # protected region user member variables begin #
        # accumulated wrenches
        self.wrenches = list()
        # number of updates without receiving any data
        self.waiting_iteration = 0
        # number of cycle
        self.cycle_id = 0
        self.looping_id = 0
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
        # todo: check the timestamps to see if the information has been updated

        # print "processing wrench: \n {}".format(data.in_wrench)
        # print "timestamp: \n {}".format(data.in_wrench.header.stamp)
        # print "Check: {}".format(type(data.in_wrench.header.stamp))
        # print "Check: {}".format(rospy.Time(0))

        self.cycle_id += 1

        if data.in_loop_updated:
            rospy.loginfo("loop_detected")
            self.looping_id +=1
            if self.looping_id > 6:
                rospy.logwarn("returning to 0")
                self.looping_id = 0

        if data.in_wrench.header.stamp == rospy.Time(0):
            self.waiting_iteration += 1

            if self.waiting_iteration %50 == 0:
                print "No data received since {} cycles".format(self.waiting_iteration)
            return

        if self.waiting_iteration != 0:
            print "Resuming recepttion after {} cycles".format(self.waiting_iteration)
            self.waiting_iteration = 0

        if not self.wrenches:
            self.wrenches.append(data.in_wrench)
        else:
            last_stamp = self.wrenches[-1].header.stamp
            current_stamp = data.in_wrench.header.stamp

            if last_stamp == current_stamp:
                # rospy.loginfo("[cycle {}] wrench not updated: {} vs {}".format(self.cycle_id,last_stamp, current_stamp))
                pass
            else:
                self.wrenches.append(data.in_wrench)

        # we currently limit the data storage to the N latest values received
        nmax = 100
        while len(self.wrenches) > nmax:
            del self.wrenches[0]

        ## generate the output
        wrench = data.in_wrench.wrench
        force = [wrench.force.x, wrench.force.y, wrench.force.z]
        torque = [wrench.torque.x, wrench.torque.y, wrench.torque.z]

        force_norm = numpy.linalg.norm(force)

        if force_norm > 1.0:
            a_x = - torque[1] / force[2]
            a_y = torque[0] / force[2]

            self.generate_plot_request(a_x, a_y)

        else:
            rospy.logwarn("Force too low: {}".format(force_norm))

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

    def generate_plot_request(self, x, y):

        request = PlotPoseRequest()
        request.msg.position.x = x
        request.msg.position.y = y
        request.append = True
        request.symbol = ord('o')
        request.symbol_size = 10
        request.series = self.looping_id
        self.passthrough.client_display(request)

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
