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


# original example provided at page:
# https://matplotlib.org/users/event_handling.html
# use to display a point, and display it around
class EnhancedGraph(object):
    def __init__(self, line, text):
        self.line = line
        self.text = text
        self.xs = list(line.get_xdata())
        self.ys = list(line.get_ydata())

        self.cid = line.figure.canvas.mpl_connect('motion_notify_event', self)

    def __call__(self, event):
        # print('click', event)
        if event.inaxes!=self.line.axes: return
        self.xs.append(event.xdata)
        self.ys.append(event.ydata)
        #self.line.set_data(self.xs, self.ys)

        msg = 'pt: {}-{}\n'.format(event.xdata, event.ydata)
        print msg
        #self.text.set_text(msg)
        self.line.set_data(event.xdata, event.ydata)
        #self.line.figure.canvas.draw()


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
        self.ax = None
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

    def init_graph(self):
        """
        @brief initialize the graph to be shown
        """

        self.fig, self.ax = self.contact_set.get_graph()

        # initializa interaction tools

        self.input, = self.ax.plot([0.1], [-0.2], 'o', color='b')

        self.cursor, = self.ax.plot([0], [0], 'o')  # empty line
        self.text = self.ax.text(0.05, 0.05, 'selected: none',
                             transform=self.ax.transAxes, va='top')
        self.enhanced_graph = EnhancedGraph(self.cursor, self.text)

        self.lines = [self.input, self.cursor, self.text]

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
        # print num
        #self.lines.set_data(0.1 + num * 0.005, -0.2)
        self.input.set_data(0.1 + num * 0.005, -0.2)
        #self.cursor.set_data(self.enhanced_graph.xs, self.enhanced_graph.ys)

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
        display.init_graph()
        display.launch_loop()
        rospy.spin()
    else:
        rospy.logerr("Prb in initialization. Bye")
