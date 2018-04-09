# !/usr/bin/env python
"""
Copyright (C) 2017 Tecnalia Research and Innovation

@package ar_contact
@author Anthony Remazeilles
@brief management of a set of contact
"""

import yaml
import os
import numpy
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

from contact_def.ar_basic_class import BasicClass
from contact_def.ar_contact_class import ContactForce

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
            self.log("[{}]: {} - {}".format(i, item.is_good_contact_, item.name_))

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
        # todo why just providing the 3 best?
        # Should class more and let the user select how many it wants 

        distances = [contact.mean_distance(x, y) for contact in self.contacts]

        index_ordered = numpy.argsort(distances)
        # self.log("distances: {}".format(distances))
        # self.log("index: {}".format(index_ordered))

        best = [[idx, distances[idx]] for idx in index_ordered]
        return best

    def sigma_order(self, x, y):

        distances = [contact.sigma_distance(x, y) for contact in self.contacts]

        index_ordered = numpy.argsort(distances)
        self.log("distances: {}".format(distances))
        self.log("index: {}".format(index_ordered))

        best = [[idx, distances[idx]] for idx in index_ordered]
        return best

    def evaluate(self, lcops):
        """
        @brief evaluate a contact defined by a set of cops
        @param self The object
        @param lcops list of cops already computed
        @return is_good, set, confidence
        """

        # define a Contact Force from the data
        request = ContactForce()
        request.cop_ = lcops
        request.characterize()
        req_type = request.check_shape()
        self.log("request type: {}".format(req_type))
        self.log("Check per distance")
        center = request.cop_mean_

        # checking the distance directly
        distances = list()
        for i, item in enumerate(self.contacts):
            distance = item.mean_distance(center[0], center[1])
            self.log("Distance: {} to {}".format(distance, item.name_))
            distances.append(distance)

        idx_dist = numpy.argsort(distances)
        self.log("ordered distances: {}".format(idx_dist))
        idx_best = idx_dist[0]
        self.log("Closest: {} {} {}".format(idx_best, distances[idx_best], self.contacts[idx_best].name_))

        th_distance = 0.005
        label = self.contacts[idx_best].name_
        # check the distance to the closest
        if distances[idx_best] < th_distance:
            confidence = 1.0
            is_good = self.contacts[idx_best].is_good_contact_
            message = "Dist. < {}".format(th_distance)
        if distances[idx_best] > 2.0 * th_distance:
            confidence = 1
            is_good = False
            message = "dist {:.2} to {} > {}".format(distances[idx_best], label, th_distance)

        if th_distance < distances[idx_best] < 2.0 * th_distance:
            confidence = 0.5
            is_good = self.contacts[idx_best].is_good_contact_
            message = "dist {:.2} to {} < 2 * {}".format(distances[idx_best], label, th_distance)

        return is_good, confidence, idx_best, message


    def full_evaluate(self, lcops):
         # define a Contact Force from the data
        request = ContactForce()
        request.cop_ = lcops
        request.characterize()
        req_type = request.check_shape()
        self.log("request type: {}".format(req_type))

        center = request.cop_mean_

        return self.check_bbox(center[0], center[1])


    def bbx_order(self, x, y, within=.9, tolerance=0):
        """
        @brief check wether the point is within the bouding box of one of the learned contacts
        @param x x coordinite of the point of interest
        @param z y coordinate of the point of interest
        @param within % of the confidence ellipse to consider
        @return list of best candidate idx with score.
        """

        dist_rec = [contact.dist_to_bbox(x, y, within, tolerance) for contact in self.contacts]

        to_print = ["{:.3}".format(item) for item in dist_rec]
        self.log("Is in score: {}".format(to_print))

        sig_order = self.euclidean_order(x, y)

        # formula to check
        best = [item for item in sig_order if dist_rec[item[0]] == 0]

        self.log("Best: {}".format(best))
        return best

    def check_bbox(self, x, y):

        self.log("bbx check")
        bbest = self.bbx_order(x, y)

        if bbest:
            confidence = 1.0
            idx_best = bbest[0][0]
            is_good = self.contacts[idx_best].is_good_contact_
            message = "center in bbox"
            return is_good, confidence, idx_best, message

        self.log("bbx check + 0.005")
        bbest = self.bbx_order(x, y, tolerance=0.005)

        if bbest:
            confidence = 0.8
            idx_best = bbest[0][0]
            is_good = self.contacts[idx_best].is_good_contact_
            message = "center in bbox + 0.005"
            return is_good, confidence, idx_best, message

        self.log("bbx check + 0.01")
        bbest = self.bbx_order(x, y, tolerance=0.01)

        if bbest:
            confidence = 0.5
            idx_best = bbest[0][0]
            is_good = self.contacts[idx_best].is_good_contact_
            message = "center in bbox + 0.01"
            return is_good, confidence, idx_best, message

        self.log_warn("No good contact")

        confidence = 0.6
        idx_best = -1
        is_good = False
        message = "Too far"
        return is_good, confidence, idx_best, message

    def evaluate_bbox(self, lcops):

        request = ContactForce()
        request.cop_ = lcops
        request.characterize()
        req_type = request.check_shape()
        self.log("request type: {}".format(req_type))
        self.log("Check per distance")
        center = request.cop_mean_

        bbest = self.bbx_order(center[0], center[1])

        self.log("Obtained in bbx:")
        for item in bbest:
            self.log("Score [{}]: [{}]".format(item[0], item [1]))

        #for i in range(3):
        #    self.log_error("Score [{}]: []".format(bbest[i][0], bbest[i][1]))


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
            plt.axis([-0.1, 0.1, -0.1, 0.1])
            #plt.axis([-2, 2, -2, 2])
        else:
            colors = get_cmap(len(self.contacts) + 1)
            plt.axis('equal')
            for i, contact in enumerate(self.contacts):
                cop = contact.cop_
                if len(cop) > 0:
                    #ax_glob.plot(cop[:, 0], cop[:, 1], 'o', color=colors(i), label="{}-{}".format(i + 1, labels[i + 1]))
                    ax_glob.plot(cop[:, 0], cop[:, 1], '.', color=colors(i), label="({}) {}".format(i, contact.name_))

                    [cop_mean, sigma, angle, major_axis, minor_axis] = contact.get_ellipse()

                    ellipse =Ellipse(cop_mean, width=major_axis,
                                     height=minor_axis, angle=angle)
                    ellipse.set_color(colors(i))
                    ellipse.set_alpha(0.6)
                    ellipse.set_clip_box(ax_glob.bbox)
                    ax_glob.add_artist(ellipse)
                else:
                    self.log_warn("No points for {}".format(i+1))
        #if self.contacts:
        #    ax_glob.legend()

        # import matplotlib.patches as mpatches
        # texts = ["Exp {}".format(i + 1) for i in range(5)]
        # patches = [ mpatches.Patch(color=colors[i], label="{:s}".format(labels[relevant_slices[i][0]]) ) for i in range(len(relevant_slices)) ]
        # plt.legend(handles=patches, ncol=2 )

        ax_glob.set_xlabel("x[m]")
        ax_glob.set_ylabel("y[m]")
        ax_glob.grid()

        if self.contacts:
            #ax_glob.legend(loc='upper center', bbox_to_anchor=(0.5, 1.0), ncol=3, fancybox=True, shadow=True)
            # to put the legend on the right
            # Shrink current axis by 20%
            box = ax_glob.get_position()
            ax_glob.set_position([box.x0, box.y0, box.width * 0.8, box.height])

            # Put a legend to the right of the current axis
            ax_glob.legend(loc='center left', bbox_to_anchor=(1, 0.5), numpoints=1)

        return fig_glob, ax_glob

if __name__ == "__main__":
    print "Hello world"

    lcontacts = ContactForceSet()
    lcontacts.set_cfg_file("/home/anthony/tmp/sarafun_contacts/config.yaml")
    lcontacts.load_contacts()

    lcontacts.get_graph()
    plt.show()