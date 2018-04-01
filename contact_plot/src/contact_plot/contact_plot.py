#!/usr/bin/env python

"""
@package contact_plot
@file contact_plot.py
@author Anthony Remazeilles
@brief python animated graph to display contact sets and current contact position
Copyright (C) 2018 Tecnalia Research and Innovation
Distributed under the GNU GPL v3.
For full terms see https://www.gnu.org/licenses/gpl.txt
"""

import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os
import yaml
from contact_def.ar_contact_set import ContactForceSet

class AnimatedContact(object):

    def __init__(self):
        """
        @brief Basic constructor

        @param      self The object
        """

        # to contain the contact set
        self.contact_set = ContactForceSet()
        # config file
        self.cfg = None

        # set of needed structures for graph management.
        # figure handler
        self.fig = None
        # animation handler
        self.anim = None
        # tbd loop function param
        self.lines = None
        # to be completed

    def load_data(self, cfg_file):

        if not self.contact_set.set_cfg_file(cfg_file):
            rospy.log_error("Prb wile setting cfg file")
            return False
        if not self.contact_set.load_contacts():
            rospy.log_error("Prb while loading data")
            return False
        return True

    def animation_loop(self, num):
        """
        @brief looping function automatically called by the animation process

        @param      self The object
        @param      num The number

        @return { description_of_the_return_value }
        """

        if rospy.is_shutdown():
            self.anim.event_sources.stop()
            plt.close(self.fig)
            return self.lines
        # update the graph
        # done
        return self.lines

    def launch_loop(self):
        """
        @brief launch of the loop

        @param      self The object

        """
        try:
            # todo check the frames parameter
            self.anim = animation.FuncAnimation(self.fig,
                                                self.animation_loop,
                                                frames=100,
                                                interval=30,
                                                blit=True,
                                                repeat=True)
            plt.show()
        except StopIteration:
            rospy.loginfo("end of the animation")
        rospy.loginfo("Bye")

if __name__ == '__main__':
    rospy.init_node('contact_plot', anonymous=True)
    display = AnimatedContact()

    cfg_file = "/home/anthony/tmp/sarafun_contacts/config.yaml"
    is_init_ok = display.load_data(cfg_file)
    if is_init_ok:
        rospy.loginfo("Ready to display data")
        #display.launch_loop()
        #rospy.spin()
    else:
        rospy.logerr("Prb in initialization. Bye")
