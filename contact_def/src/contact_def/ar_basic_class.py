"""
Copyright (C) 2017 Tecnalia Research and Innovation

@package ar_contact
@author Anthony Remazeilles
@brief Enhancement of a class for easier debugginf
"""

from termcolor import colored
import inspect
import subprocess
import sys


class BasicClass(object):
    """ Common methods for any class
    """
    def __init__(self, name="BasicClass"):
        """ Defining basic and common functions
        name: name of the class
        """
        self.name_ = name

    def log(self, text):
        """ display log message with the class name in parameter
        text the string to display
        """
        print "[{}::{}] ".format(self.name_, inspect.stack()[1][3]) + text


    def log_warn(self, text):
        """ display warn message with the class name in parameter
        text the string to display
        """
        print colored("[{}::{}] ".format(self.name_, inspect.stack()[1][3]) + text, 'yellow')


    def log_error(self, text):
        """ display warn message with the class name in parameter
        text the string to display
        """
        print colored("[{}::{}] ".format(self.name_, inspect.stack()[1][3]) + text, 'red')


    def get_git_info(self):
        """
        get some information about the git repository this code is in it
        :return: [branch-name, hash-key]
        :warning just call form the current repo, may not be ok always
        """
        try:
            git_hash = subprocess.check_output(["git", "describe", "--always"])
            git_hash = git_hash.replace("\n", "")
            git_branch = subprocess.check_output(["git", "rev-parse", "--abbrev-ref", "HEAD"])
            git_branch = git_branch.replace("\n", "")
        except:
            self.log_error("Prb while acceding to the git information")
            self.log_error(str(sys.exc_info()[0]))
            return []
        return [git_branch, git_hash]
