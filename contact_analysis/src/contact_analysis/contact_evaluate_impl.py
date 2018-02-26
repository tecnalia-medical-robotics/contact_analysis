#!/usr/bin/env python
"""
@package contact_analysis
@file contact_evaluate_impl.py
@author Anthony Remazeilles
@brief Analysis of the Center of Pressure related to contact points

Copyright (C) {packageCopyright}
https://www.gnu.org/licenses/gpl.txt
"""

import rospy
from geometry_msgs.msg import Point
from contact_msgs.msg import LearnContactAction, LearnContactActionFeedback, LearnContactActionResult
from contact_msgs.msg import EvaluateContactAction, EvaluateContactActionFeedback, EvaluateContactActionResult

# protected region user include package begin #
# protected region user include package end #

class ContactEvaluateConfig(object):
    """
    set of elements accessible through dynamic reconfigure
    autogenerated: don't touch this class
    """
    def __init__(self):
        pass

    def __str__(self):
        msg = "Instance of ContactEvaluateConfig class: {"
        msg += "}"
        return msg

class ContactEvaluateData(object):
    """
    set of input / output handled through the update methods
    autogenerated: don't touch this class
    """
    def __init__(self):
        """
        Definition of the ContactEvaluateData attributes
        """
        # input data
        self.in_cop = Point()
        self.in_cop_updated = bool()
        pass

    def __str__(self):
        msg = "Instance of ContactEvaluateData class: \n {"
        msg += "in_cop: {} \n".format(self.in_cop)
        msg += "in_cop_updated: {} \n".format(self.in_cop_updated)
        msg += "}"
        return msg

class ContactEvaluatePassthrough(object):
    """
    set of passthrough elements slightly violating interface / implementation separation
    Autogenerated: don't touch this class
    """
    def __init__(self):
        """ Class to contain variable breaking the interface separation
        """
        self.as_learn = None
        self.as_evaluate = None
        pass

class ContactEvaluateImplementation(object):
    """
    Class to contain Developer implementation.
    """
    def __init__(self):
        """
        Definition and initialisation of class attributes
        """
        self.passthrough = ContactEvaluatePassthrough()

        # protected region user member variables begin #
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
        # protected region user update end #
        pass

    def callback_learn(self, goal):
        """
        @brief callback of service learn

        @param self The object
        @param goal(LearnContact) goal provided

        @return (LearnContactResponse) service output
        @warning may send some feedback during the task execution
        """

        # to provide feedback during action execution
        # to send the feedback, one should use:
        # self.passthrough.as_learn.publish_feedback(feedback)
        feedback = LearnContactActionFeedback()
        # to contain the outcome of the task at completion
        # to send the result, one should use:
        # on suceess:
        # self.passthrough.as_learn.set_succeeded(result)
        result = LearnContactActionResult()
        # Remind that preemption request should be checked during action execution:
        # if self.passthrough.as_learn.is_preempt_requested():
        #        rospy.loginfo('Preempted action learn')
        #        self.passthrough.as_learn.set_preempted()
        #        success = False
        #        break

        # protected region user implementation of action callback for learn begin #
        # protected region user implementation of action callback for learn end #

    def callback_evaluate(self, goal):
        """
        @brief callback of service evaluate

        @param self The object
        @param goal(EvaluateContact) goal provided

        @return (EvaluateContactResponse) service output
        @warning may send some feedback during the task execution
        """

        # to provide feedback during action execution
        # to send the feedback, one should use:
        # self.passthrough.as_evaluate.publish_feedback(feedback)
        feedback = EvaluateContactActionFeedback()
        # to contain the outcome of the task at completion
        # to send the result, one should use:
        # on suceess:
        # self.passthrough.as_evaluate.set_succeeded(result)
        result = EvaluateContactActionResult()
        # Remind that preemption request should be checked during action execution:
        # if self.passthrough.as_evaluate.is_preempt_requested():
        #        rospy.loginfo('Preempted action evaluate')
        #        self.passthrough.as_evaluate.set_preempted()
        #        success = False
        #        break

        # protected region user implementation of action callback for evaluate begin #
        # protected region user implementation of action callback for evaluate end #

    # protected region user additional functions begin #
    # protected region user additional functions end #
