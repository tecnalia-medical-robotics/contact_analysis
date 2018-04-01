# !/usr/bin/env python
"""
Copyright (C) 2017 Tecnalia Research and Innovation

@package ar_contact
@author Anthony Remazeilles
@brief management of a set of contact
"""

import yaml
import os
from ar_basic_class import BasicClass
from ar_contact_class import ContactForce

class ContactForceSet(BasicClass):
    """
    @brief managing a set of classes
    """
    def __init__(self, name="contactSet"):
        """
        Parameters initialization
        """
        super(ContactForceSet, self).__init__(name)

        # to contain the list of contacts
        self.contacts = None
        # to contain the configuration file
        self.cfg = None

    def set_cfg_file(self, config_file):
        """
        @brief set the path to the set of files
        @param      self The object
        @param      dirname name of the directory to load

        @return True on success
        """
        self.log("config file {}".format(config_file))

        # confirm string is a valid path
        if not os.path.isfile(config_file):
            return False

        with open(config_file, 'r') as file_c:
            self.cfg = yaml.load(file_c)

        # check if absolute path or not.
        if os.path.isabs(self.cfg["filepattern"]):
            return True

        # path is absolute, so we add the cfg file path
        path_abs = os.path.dirname(config_file)
        self.log("Abs path: {}".format(path_abs))
        self.log("Adding absolute path to file pattern")
        self.cfg['filepattern'] = path_abs + '/' + self.cfg['filepattern']

        self.log("New file pattern: {}".format(self.cfg['filepattern']))

        return True

    def load_contacts(self, cfg_file=None):
        """
        @brief load the contacts
        @param      self The object
        @param      cfg_file The configuration file.
        @return True on sucess
        """
        # todo check if this is a dictionary.
        # if it is load it

        self.contacts = list()

        if cfg_file is not None:
            if not self.set_cfg_file(cfg_file):
                return False

        self.log("Loading contact models")

        for idx in self.cfg['indexes']:
            filename = self.cfg['filepattern'].format(idx)
            self.log("Loading file {}".format(filename))
            contact = ContactForce()
            contact.load_object(filename)
            self.contacts.append(contact)

        # check the files loaded
        for i, item in enumerate(self.contacts):
            self.log("set {} - {} - {}".format(i, item.name_, item.cop_))
        return True



if __name__ == "__main__":
    print "Hello world"

    lcontacts = ContactForceSet()
    lcontacts.set_cfg_file("/home/anthony/tmp/sarafun_contacts/config.yaml")
    lcontacts.load_contacts()