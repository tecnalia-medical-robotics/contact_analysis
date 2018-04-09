# !/usr/bin/env python
"""
Copyright (C) 2017 Tecnalia Research and Innovation

@package ar_contact
@author Anthony Remazeilles
@brief Contact characterization class
"""

import numpy
import math
import yaml
import pickle
from scipy import stats
from contact_def.ar_basic_class import BasicClass

class ContactForce(BasicClass):
    """
    Class related to a contact experiment
    """

    def __init__(self, name="contact", is_good_contact=True, min_force=None):
        """
        Parameters initialization
        """
        # call super class constructor
        super(ContactForce, self).__init__(name)
        # to know wether the contact is consodered to be good
        self.is_good_contact_ = True
        # to contain all data, to be a numpy array
        self.data_ = None
        # to contain the cop, numpy array
        self.cop_ = None
        # to contain the area mean
        self.cop_mean_ = None
        # to contain the angle between the two axis
        self.sigma_ = None
        # to contain the angle between axes
        self.angle_ = None
        # to contain the singular vectors
        self.sing_vectors_ = None

        # minimum force to keep contact in set
        if min_force:
            self.min_force_ = min_force
        else:
            self.min_force_ = 0

    def reset(self):
        """
        @brief reset all variables
        """
        self.data_ = None
        self.cop_ = None
        self.cop_mean_ = None
        self.sigma_ = None
        self.angle_ = None
        self.sing_vectors_ = None

    def set_wrenches(self, wrench_array):
        """
        @brief Sets the wrenches.

        @param      self The object
        @param      wrench_array The array of wrenches
                    assumed [*force *torque]
        @return True if ok
        """

        self.reset()

        _, cols = wrench_array.shape

        if cols != 6:
            self.log_error("input data should have 7 columns")
            self.log_error("provided: {}".format(wrench_array.shape))
            return False

        self.data_ = wrench_array
        return True

    def set_cops(self, cop_array):
        """
        @brief set directly the cop set
        @param self The object
        @param cop_array array of cops
        @return True on sucess
        """

        self.reset()
        rows, cols = cop_array.shape

        if cols != 2:
            self.log_error("input data should have 2 columns")
            self.log_error("provided: {}".format(cop_array.shape))
            return False

        self.cop_ = cop_array
        return True

    def compute_cop(self, min_force=None):
        """
        @brief compute The Center of Pressure

        @param      self The object
        @param      min_force The minimum force accepted.
                    if unset, the default value is used.

        @return number of cop points kept, -1 if an error occured
        """

        if self.data_ is None:
            return -1

        if min_force is None:
            min_force = self.min_force_

        force = self.data_[:, 0:3]
        torque = self.data_[:, 3:6]

        total_sample, _ = force.shape

        cop = []
        for iforce, itorque in zip(force, torque):
            force_norm = numpy.linalg.norm(iforce)

            #self.log("force: {}".format(iforce))
            if (force_norm > min_force) and (iforce[2] != 0.0):
                a_x = - itorque[1] / iforce[2]
                a_y = itorque[0] / iforce[2]
                cop.append([a_x, a_y])

        nb_cop = len(cop)
        if nb_cop != 0:
            self.cop_ = numpy.asarray(cop)
            self.log("Number of samples above thresold: {}/{}".format(len(cop),
                                                                      total_sample))
        else:
            self.log_warn("No samples above threshold")
            self.cop_ = None

        return nb_cop

    def characterize(self):
        """
        @brief characterize the set, computing a svd.
        @param      self The object
        @return true if the operation succeeded.
        """

        if self.cop_ is None:
            return False

        # todo set a reference
        cop_mean = self.cop_.mean(axis=0)
        self.log("cop_mean: {}".format(cop_mean))
        cop_m = self.cop_ - cop_mean

        # self.log("data shape: {}".format(cop_m.shape))

        U, sing_values, VT = numpy.linalg.svd(cop_m, full_matrices=False)
        V = VT.T
        self.log('Singular values : \n{}'.format(sing_values))
        self.log('Singular vectors : {}'.format(V))
        #self.log('V.T is : {}'.format(V.T))

        # projecting point into the space spanned by the eigenvectors
        # to get the distribution on the axes
        projected_cop = numpy.dot(cop_m, V)
        # self.log("projected_cop shape: {}".format(projected_cop.shape))

        sigma = projected_cop.std(axis=0)
        self.log("sigma is: {}".format(sigma))
        self.log("sigma max: {}".format(sigma.max()))
        self.log("sigma min: {}".format(sigma.min()))

        # computing the angle between the two axes of V
        angle_rad = math.atan2(V[1, 0], V[0, 0])
        angle_deg = math.degrees(angle_rad)

        self.log('Angle: {}'.format(angle_deg))
        if angle_deg < 0:
            angle_deg += 360.0
        self.log('Angle: {}'.format(angle_deg))

        # Storing the area information
        self.cop_mean_ = cop_mean
        self.sigma_ = sigma
        self.angle_ = angle_deg
        self.sing_vectors_ = V
        return True

    def get_ellipse(self, within=.90):
        """
        @brief compute the ellipsoids
        see details on:
        http://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/

        @param      self The object
        @param      within % of data to be in the ellipsoids
        @return The ellipse.
        """
        assert 0.0 < within < 1.0

        # should be aligned with Chi Square probabilities
        # see https://people.richland.edu/james/lecture/m170/tbl-chi.html
        s_chi = stats.chi2.isf(1.0 - within, 2)
        major_axis = 2.0 * math.sqrt(s_chi) * self.sigma_.max()
        minor_axis = 2.0 * math.sqrt(s_chi) * self.sigma_.min()

        return [self.cop_mean_, self.sigma_, self.angle_, major_axis, minor_axis]

    def mean_distance(self, x, y):
        """ Compute euclidean distance to the mean point

        Arguments:
            x {float} -- 1st coordinate
            y {float} -- 2nd coordinate

        Returns:
            [float] -- Computed distance, -1 if can not be computed
        """
        if self.cop_mean_ is None:
            return -1
        if self.sigma_ is None:
            return -1

        pt = numpy.array([x, y])

        distance = numpy.linalg.norm(pt - self.cop_mean_)

        return distance

    def sigma_distance(self, x, y):
        """Compute euclidean distance scaled by the std

        x {float} -- 1st coordinate
        y {float} -- 2nd coordinate

        Returns:
            [float] -- Computed distance, -1 on error
        """

        if self.cop_mean_ is None:
            return -1
        if self.sigma_ is None:
            return -1

        pt = numpy.array([x, y]) - self.cop_mean_

        aligned_pt = numpy.dot(pt,  self.sing_vectors_)

        sigma_pt = aligned_pt / self.sigma_

        distance = numpy.linalg.norm(sigma_pt)

        return distance

    def is_pt_in_bbox(self, x, y , within=.90, tolerance=0):
        """
        @brief check wether a point is within the contact definition area
        @param self the object
        @param x x coordinates of the request point
        @param y y coordinates of the request point
        @param within % of data to be represented by the ellipse
        @warning the ellipse is approximated by a bbox
        """

        if self.cop_mean_ is None:
            return False
        if self.sigma_ is None:
            return False

        pt = numpy.array([x, y]) - self.cop_mean_
        aligned_pt = numpy.dot(pt,  self.sing_vectors_)

        # todo: check the ellipse size: we may have to use half of it.
        _, _, _, max_axis, min_axis = self.get_ellipse(within)

        x_b = max_axis / 2.0 + tolerance
        y_b = min_axis / 2.0 + tolerance

        print "pt projected: {}".format(aligned_pt)
        print "From [{}, {}] we generate [{}, {}]".format(self.sigma_[0],
                                                          self.sigma_[1],
                                                          x_b, y_b)

        is_in = (-x_b < aligned_pt[0] < x_b) and (-y_b < aligned_pt[1] < y_b)

        return  is_in

    def dist_to_bbox(self, x, y , within=.90, tolerance=0):

        if self.cop_mean_ is None:
            return False
        if self.sigma_ is None:
            return False

        pt = numpy.array([x, y]) - self.cop_mean_
        aligned_pt = numpy.dot(pt,  self.sing_vectors_)

        # todo: check the ellipse size: we may have to use half of it.
        _, _, _, max_axis, min_axis = self.get_ellipse(within)

        x_b = max_axis / 2.0 + tolerance
        y_b = min_axis / 2.0 + tolerance

        dx = max(abs(aligned_pt[0]) - x_b, 0.0)
        dy = max(abs(aligned_pt[1]) - y_b, 0.0)
        return dx * dx + dy * dy

    def check_shape(self):
        """
        Check the shape of the contact
        """

        # Look at the ratio between the two sigma
        #assert self.sigma_ is not None

        ratio = self.sigma_[0] / self.sigma_[1]
        self.log("sigma: [{:.03} {:.03}] ratio: {}".format(self.sigma_[0], self.sigma_[1], ratio))

        if ratio >= 9:
            ratio_type = "Line"
        else:
            ratio_type = "Point"

        self.log_warn(ratio_type)
        return ratio_type


    def store_data_yaml(self, filename):
        """
        @brief Stores spec into a file.

        @param      self The object
        @param      filename The filename to store the file

        @return True on success.
        @todo look at the pickle operation
        """

        # generates the disctionnary of data
        cfg = dict()

        if self.data_ is not None:
            cfg['data'] = self.data_
        if self.cop_ is not None:
            cfg['cop_'] = self.cop_
        if self.cop_mean_ is not None:
            cfg['cop_mean'] = self.cop_mean_
        if self.sigma_ is not None:
            cfg['sigma'] = self.sigma_
        if self.angle_ is not None:
            cfg['angle'] = self.angle_
        if self.sing_vectors_ is not None:
            cfg['sing_vectors'] = self.sing_vectors_
        cfg['min_force'] = self.min_force_
        cfg['name'] = self.name_
        cfg['is_good_contact'] = self.is_good_contact_
        with open(filename, 'w') as outfile:
            yaml.dump(cfg, outfile, default_flow_style=False)
        return True

    def dump_object(self, filename):
        """
        @brief Dump the state of the object.

        @param      self The object
        @param      filename The filename

        @return True on success
        @todo add exception handling
        """

        pickle.dump(self,open(filename,'w'))

        return True

    def load_object(self, filename):
        """
        @brief initialize by reading a dumped object

        @param      self The self
        @param      filename The filename to read

        @return True on success
        @todo add exception handling
        """

        obj = pickle.load(open(filename))
        self.__dict__ = obj.__dict__
        return True
