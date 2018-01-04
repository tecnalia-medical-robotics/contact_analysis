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
# todo do not need both of them

# protected region user include files begin #
import csv
import numpy
# protected region user include files end #

class wrench_from_csvConfig(object):
    """
    set of elements accessible through dynamic reconfigure
    """
    def __init__(self):
        self.csv_file = "Undef"
        pass

    def __str__(self):
        msg = "Instance of wrench_from_csvConfig class: {"
        msg += "csv_file: {} ".format(self.csv_file)
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
        pass

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
        # self.id_wrench = -1
        self.id_wrench = 5000
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
                    data = row_array[13:19]

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
        self.id_wrench += 1
        if self.id_wrench >= len(self.wrenches):
            rospy.loginfo("Published all wrenches, looping at next iteration")
            #self.id_wrench = -1
            self.id_wrench = 5000
            data.out_wrench_active = False
            return
        wrench = self.wrenches[self.id_wrench]
        wrench.header.stamp = rospy.get_rostime()
        data.out_wrench = wrench
        # protected region user update end #
        pass

    # protected region user additional functions begin #
    # protected region user additional functions end #
