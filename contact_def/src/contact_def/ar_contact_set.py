# !/usr/bin/env python
"""
Copyright (C) 2017 Tecnalia Research and Innovation

@package ar_contact
@author Anthony Remazeilles
@brief management of a set of contact
"""

import yaml
import os
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

from ar_basic_class import BasicClass
from ar_contact_class import ContactForce

def get_cmap(n, name='hsv'):
    '''Returns a function that maps each index in 0, 1, ..., n-1 to a distinct
    RGB color; the keyword argument name must be a standard mpl colormap name.
    For changeing the map, see the following link:
    https://matplotlib.org/1.3.1/examples/color/colormaps_reference.html
    '''
    return plt.cm.get_cmap(name, n)


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
        self.contacts = list()
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
            self.log_error("File provided does not exist")
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

        self.log("Loaded {} models".format(len(self.contacts)))
        # check the files loaded
        for i, item in enumerate(self.contacts):
            #self.log("set {} - {} - {}".format(i, item.name_, item.cop_))
            self.log("set {} - {}".format(i, item.name_))

        return True

    def store_contacts(self, dir_name, cfg_file = None):
        """
        @brief load the contacts
        @param      self The object
        @param      cfg_file The configuration file.
        @return True on sucess
        """
        # todo check if this is a dictionary.
        # if it is load it

        if not self.contacts:
            self.log_error("no contact defined. Bye")
            return False

        if cfg_file is None:
            cfg_file = "config.yaml"
            self.log_warn("Config filename set to {}".format(cfg_file))

        # check existence of the path provided
        if os.path.exists(dir_name) and os.path.isdir(dir_name):
            self.log_warn("Directory already exist.")

        # self.log("Here")
        if not os.path.exists(dir_name):
            self.log("Crreating the directory {}".format(dir_name))
            os.makedirs(dir_name)

        # self.log("There")

        filepattern = "contact_{}_picle.yaml"
        full_filepattern = dir_name + "/"  + filepattern
        for i, item in enumerate(self.contacts):
            item.dump_object(full_filepattern.format(i))

        # creating now the config file
        config = dict()
        config["filepattern"] = filepattern
        config["indexes"] = range(len(self.contacts))

        filename = dir_name + "/" + cfg_file
        with open(filename, 'w') as outfile:
            yaml.dump(config, outfile, default_flow_style=False)

        return True

    def euclidean_order(self, x, y):

        distances = [contact.mean_distance(x, y) for contact in self.contacts]

        index_ordered = numpy.argsort(distances)
        self.log("distances: {}".format(distances))
        self.log("index: {}".format(index_ordered))

        # select the best 3
        best = list()

        for i in range(3):
            idx = index_ordered[i]
            best.append([idx, distances[idx]])
        return best

    def sigma_order(self, x, y):

        distances = [contact.sigma_distance(x, y) for contact in self.contacts]

        index_ordered = numpy.argsort(distances)
        self.log("distances: {}".format(distances))
        self.log("index: {}".format(index_ordered))

        # select the best 3
        best = list()

        for i in range(3):
            idx = index_ordered[i]
            best.append([idx, distances[idx]])
        return best

    def evaluate(self, lcops):
        """
        @brief evaluate a contact defined by a set of cops
        @param self The object
        @param lcops list of cops already computed
        @return is_good, set, confidence
        """
        # to be defined...
        pass


    def get_graph(self):
        """
        @brief generate the graph of the contacts
        @param      self The object
        @return The graph.
        @todo add a label to the sets
        """

        fig_glob, ax_glob = plt.subplots()
        ax_glob.set_title("Global COP Analysis")

        if not self.contacts:
            plt.axis([-0.5, 0.5, -0.5, 0.5])
        else:
            colors = get_cmap(len(self.contacts) + 1)

            for i, contact in enumerate(self.contacts):
                cop = contact.cop_
                if len(cop) > 0:
                    #ax_glob.plot(cop[:, 0], cop[:, 1], 'o', color=colors(i), label="{}-{}".format(i + 1, labels[i + 1]))
                    ax_glob.plot(cop[:, 0], cop[:, 1], '.', color=colors(i), label="{}".format(i))

                    [cop_mean, sigma, angle, major_axis, minor_axis] = contact.get_ellipse()

                    ellipse =Ellipse(cop_mean, width=major_axis,
                                     height=minor_axis, angle=angle)
                    ellipse.set_color(colors(i))
                    ellipse.set_alpha(0.6)
                    ellipse.set_clip_box(ax_glob.bbox)
                    ax_glob.add_artist(ellipse)
                else:
                    self.log_warn("No points for {}".format(i+1))
        ax_glob.legend()

        # import matplotlib.patches as mpatches
        # texts = ["Exp {}".format(i + 1) for i in range(5)]
        # patches = [ mpatches.Patch(color=colors[i], label="{:s}".format(labels[relevant_slices[i][0]]) ) for i in range(len(relevant_slices)) ]
        # plt.legend(handles=patches, ncol=2 )

        ax_glob.set_xlabel("x[mm]")
        ax_glob.set_ylabel("y[mm]")
        ax_glob.grid()
        ax_glob.legend(loc='upper center', bbox_to_anchor=(0.5, 1.0), ncol=3, fancybox=True, shadow=True)

        return fig_glob, ax_glob

if __name__ == "__main__":
    print "Hello world"

    lcontacts = ContactForceSet()
    lcontacts.set_cfg_file("/home/anthony/tmp/sarafun_contacts/config.yaml")
    lcontacts.load_contacts()

    lcontacts.get_graph()
    plt.show()