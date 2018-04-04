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
from contact_def.ar_contact_set import ContactForceSet

from geometry_msgs.msg import Point
from contact_msgs.srv import SetString
from contact_msgs.srv import SetString, SetStringResponse


# original example provided at page:
# https://matplotlib.org/users/event_handling.html
# use to display a point, and display it around
class EnhancedGraph(object):
    def __init__(self, line, text):
        self.line = line
        self.text = text

        self.cid = line.figure.canvas.mpl_connect('motion_notify_event', self)

    def __call__(self, event):
        # print('click', event)
        if event.inaxes != self.line.axes:
            return

        msg = 'pt: [{:.3}, {:.3}]'.format(event.xdata, event.ydata)
        # print msg
        self.text.set_text(msg)
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
        # to get the current cop measurement.
        self.sub_cop = None
        # to contain the last cop received
        self.last_cop = None
        # to receive the contact set repo
        self.srv_load = rospy.Service("plot_load", SetString, self.srv_load_callback)

        # set of needed structures for graph management.
        # figure handler
        self.fig = None
        self.ax = None
        # animation handler
        self.anim = None
        # tbd loop function param
        self.lines = None
        # to be completed

    def srv_load_callback(self, req):
        result = SetStringResponse()
        result.success = self.contact_set.load_contacts(req.message)

        if result.success:
            rospy.logwarn("Stopping animation")
            self.anim.event_source.stop()
            plt.close(self.fig)
        else:
            rospy.logerr("Could not update the contact set")
        return result

    def load_data(self, cfg_file):

        if not self.contact_set.set_cfg_file(cfg_file):
            rospy.logerr("Prb wile setting cfg file")
            return False
        if not self.contact_set.load_contacts():
            rospy.logerr("Prb while loading data")
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
        self.text_cb = self.ax.text(0.05, 0.09, 'sel: none',
                                    transform=self.ax.transAxes, va='top')
        self.text_cursor = self.ax.text(0.05, 0.05, 'cb : none',
                                        transform=self.ax.transAxes, va='top')

        self.enhanced_graph = EnhancedGraph(self.cursor, self.text_cursor)

        self.lines = [self.input, self.cursor, self.text_cursor, self.text_cb]

    def topic_callback_cop(self, msg):
        """
        @brief ros callback on message
        @param self the object
        @param msg received message [geometry_msgs/Point]
        """
        # rospy.loginfo("Received msg: {}".format(msg))
        self.last_cop = msg

    def animation_loop(self, num):
        """
        @brief looping function automatically called by the animation process

        @param      self The object
        @param      num The number

        @return { description_of_the_return_value }
        """
        if rospy.is_shutdown():
            self.anim.event_source.stop()
            plt.close(self.fig)
            return self.lines

        # update the graph
        if self.last_cop is not None:
            self.input.set_data(self.last_cop.x, self.last_cop.y)
            msg = 'cb : [{:.3}, {:.3}]'.format(self.last_cop.x, self.last_cop.y)
            #print msg
            self.text_cb.set_text(msg)

        # done
        return self.lines

    def launch_loop(self):
        """
        @brief launch of the loop

        @param      self The object

        """

        self.sub_cop = rospy.Subscriber('cop', Point, self.topic_callback_cop)

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

    def launch_infinite_loop(self):
        """
        @brief launch of the loop

        @param      self The object

        """

        self.sub_cop = rospy.Subscriber('cop', Point, self.topic_callback_cop)

        while True:
            if rospy.is_shutdown():
                rospy.logwarn("Shutting down")
                return
            self.init_graph()

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
            rospy.logwarn("looping")
        rospy.loginfo("Bye")

if __name__ == '__main__':
    rospy.init_node('contact_plot', anonymous=True)
    display = AnimatedContact()

    cfg_file = "/home/anthony/tmp/sarafun_contacts/config.yaml"
    #is_init_ok = display.load_data(cfg_file)
    is_init_ok = True
    if is_init_ok:
        rospy.loginfo("Ready to display data")
        # display.init_graph()
        #display.launch_loop()
        display.launch_infinite_loop()
        rospy.loginfo("loop closed")
        rospy.spin()
    else:
        rospy.logerr("Prb in initialization. Bye")
