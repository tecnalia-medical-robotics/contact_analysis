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
import numpy
import os

class AnimatedContact(object):

    def __init__(self):
        """
        @brief Basic constructor

        @param      self The object
        """

        # to contain the contact set
        self.lcontacts = None
        # set of needed structures for graph management.
        # figure handler
        self.fig = None
        # animation handler
        self.anim = None
        # tbd loop function param
        self.lines = None
        # to be done

        # whether initialization is ok
        self.is_init_ok = False

    def def_dir_path(self, dirname):
        """
        @brief set the path to the set of files
        @param      self The object
        @param      dirname name of the directory to load

        @return True on success
        """

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

    if display.is_init_ok:
        display.loop()
        rospy.spin()
    else:
        rospy.logerr("Prb in initialization. Bye")