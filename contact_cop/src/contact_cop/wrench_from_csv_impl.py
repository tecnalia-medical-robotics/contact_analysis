#!/usr/bin/env python
"""
@package contact_cop
@file wrench_from_csv_impl.py
@author Anthony Remazeilles
@brief Analysis of the Center of Pressure related to contact points

Copyright (C) {packageCopyright}
https://www.gnu.org/licenses/gpl.txt
"""

import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray
# todo do not need both of them

# protected region user include files begin #
import csv
import numpy
from visualization_msgs.msg import Marker
import simplejson
# protected region user include files end #

class wrench_from_csvConfig(object):
    """
    set of elements accessible through dynamic reconfigure
    """
    def __init__(self):
        self.csv_file = "Undef"
        self.inc = 1
        self.slice_file = "Undef"
        self.label_file = "Undef"
        pass

    def __str__(self):
        msg = "Instance of wrench_from_csvConfig class: {"
        msg += "csv_file: {} ".format(self.csv_file)
        msg += "inc: {} ".format(self.inc)
        msg += "slice_file: {} ".format(self.slice_file)
        msg += "label_file: {} ".format(self.label_file)
        msg += "}"
        return msg

class wrench_from_csvData(object):
    """
    set of input / output handled through the update methods
    """
    def __init__(self):
        """
        Definition of the wrench_from_csvData attributes
        """
        # autogenerated: don't touch this class
        # output data
        self.out_wrench = WrenchStamped()
        self.out_wrench_active = bool()
        self.out_loop = Bool()
        self.out_loop_active = bool()
        self.out_data_info = MarkerArray()
        self.out_data_info_active = bool()
        pass

    def __str__(self):
        msg = "Instance of wrench_from_csvData class: \n {"
        msg += "out_wrench: {} \n".format(self.out_wrench_active)
        msg += "out_wrench_active: {} \n".format(self.out_wrench_active)
        msg += "out_loop: {} \n".format(self.out_loop_active)
        msg += "out_loop_active: {} \n".format(self.out_loop_active)
        msg += "out_data_info: {} \n".format(self.out_data_info_active)
        msg += "out_data_info_active: {} \n".format(self.out_data_info_active)
        msg += "}"
        return msg

class wrench_from_csvPassthrough(object):
    """
    set of passthrough elements slightly violating interface / implementation separation
    """
    def __init__(self):
        """ Class to contain variable breaking the interface separation
        """
        pass

class wrench_from_csvImplementation(object):
    """
    Class to contain Developer implementation.
    """
    def __init__(self):
        """
        Definition and initialisation of class attributes
        """
        self.passthrough = wrench_from_csvPassthrough()

        # protected region user member variables begin #
        # list of wrenches read
        self.wrenches = list()
        # id of the last wrench published

        self.id_wrench = -1
        self.marker = create_marker_text("ref", "map")
        self.slices = list()
        self.label = dict()

        # protected region user member variables end #

    def configure(self, config):
        """
        @brief configuration of the implementation
        @param      self The object
        @param      config set of configuration parameters
        @return True on success
        """
        # protected region user configure begin #
        rospy.loginfo("opening file {}".format(config.csv_file))

        try:
            with open(config.csv_file, 'rb') as file_handler:
                reader = csv.reader(file_handler, delimiter=";")
                # skip the header
                next(reader, None)
                for row in reader:
                    row_array = numpy.asarray(row)
                    row_array[row_array == ''] = '0'
                    # todo: set it as a configuration parameter
                    # data = row_array[13:19]
                    data = row_array[0:6]

                    wrench_stamped = WrenchStamped()
                    wrench_stamped.header.frame_id = "force_sensor"

                    # rospy.loginfo("Read: {}".format(data))
                    wrench_stamped.wrench.force.x = float(data[0])
                    wrench_stamped.wrench.force.y = float(data[1])
                    wrench_stamped.wrench.force.z = float(data[2])
                    wrench_stamped.wrench.torque.x = float(data[3])
                    wrench_stamped.wrench.torque.y = float(data[4])
                    wrench_stamped.wrench.torque.z = float(data[5])
                    self.wrenches.append(wrench_stamped)

        except IOError as error:
            rospy.logerr("Prb while loading file")
            rospy.logerr("Error: {}".format(error))
            return False

        rospy.loginfo("Loaded {} wrenches".format(len(self.wrenches)))

        if config.slice_file != "undef":
            rospy.loginfo("Loading slice file")
            try:
                with open(config.slice_file) as f:
                        self.slices = simplejson.load(f)
            except (IOError, simplejson.scanner.JSONDecodeError) as error:
                rospy.logerr("Prb while loading file")
                rospy.logerr("Error: {}".format(error))
                self.slices = list()

        if config.label_file != "undef":
            rospy.loginfo("Loading label file")
            try:
                with open(config.label_file) as f:
                    self.labels=eval(f.read())
            except IOError as error:
                rospy.logerr("Prb while loading file")
                rospy.logerr("Error: {}".format(error))
                self.labels = dict()

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

        # rospy.loginfo("Check : inc is {} ".format(config.inc))
        self.id_wrench += config.inc
        data.out_loop_active = False
        if self.id_wrench >= len(self.wrenches):
            rospy.loginfo("Published all wrenches, looping at next iteration")
            self.id_wrench = - config.inc

            data.out_wrench_active = False
            data.out_loop = True
            data.out_loop_active = True
            return
        wrench = self.wrenches[self.id_wrench]
        wrench.header.stamp = rospy.get_rostime()
        data.out_wrench = wrench


        idx_label = get_relevant_slices(self.id_wrench, self.slices)
        if idx_label in self.labels:
            label = self.labels[idx_label]
        else:
            label = "nop"
        self.marker.header.stamp = wrench.header.stamp
        self.marker.text = "data {} {}".format(self.id_wrench, label)
        data.out_data_info.markers = [self.marker]
        # protected region user update end #
        pass

    # protected region user additional functions begin #
def create_marker_text(ns='debug', frame_id='map'):
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
    point_marker.type = Marker.TEXT_VIEW_FACING
    point_marker.action = Marker.ADD
    point_marker.color.r = 1.0
    point_marker.color.g = 1.0
    point_marker.color.b = 1.0

    point_marker.color.a = 1.0
    point_marker.scale.x = 0.005
    point_marker.scale.y = 0.005
    point_marker.scale.z = 0.005
    point_marker.lifetime = rospy.Duration(0.1)
    return point_marker

def get_relevant_slices(idx, slices):
    for oneslice in slices:
        if idx >= oneslice[1] and idx <= oneslice[2]:
            return oneslice[0]
    return -1
    # protected region user additional functions end #
