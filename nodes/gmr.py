#! /usr/bin/env python
"""
Author: Jeremy M. Stober
Program: GMM.PY
Date: Friday, June 24 2011
Description: An experimental python replacement for fgmm and ds_node.
"""

import roslib
roslib.load_manifest('seds')
import rospy
from seds.srv import DSLoad, DSSrv, DSLoadResponse, DSSrvResponse

import numpy as np
import numpy.linalg as la
import numpy.random as npr
npa = np.array

import sys
import pdb

class Normal(object):
    """
    A class for storing the parameters of a conditional multivariate
    normal distribution. Pre-computes as many parameters as possible
    for fast pdf estimation.
    """

    def __init__(self, idim, mu, sigma):
        self.idim = idim # dimension of the inputs for the conditional distribution
        self.mu = mu
        self.sigma = sigma
        self.subs =

        Eaa = sigma[:d,:d]
        Ebb = sigma[d:,d:]
        Eab = sigma[:d,d:]
        Eba = sigma[d:,:d]

        # we are going to be computing b | a conditional probabilities
        # b : dx
        # a : x

        iEaa = la.inv(Eaa)
        det = la.det(Eaa)
        Ebca = Ebb - np.dot(np.dot(Eba, iEaa), Eab) # from Bishop pg. 87
        factor = (2.0 * np.pi)**(self.idim / 2.0) * (det)**(0.5)

        self.subs.append({'E' : sigma, 'dEaa' : det, 'fEaa' : factor,
                          'Eaa' : Eaa, 'Ebb' : Ebb,
                          'Eab' : Eab, 'Eba' : Eba,
                          'iEaa' : iEaa, 'Ebca' : Ebca})

    def cpdf(self, x):

        iEaa = self.subs['iEaa']
        fEaa = self.subs['fEaa']

        answer = np.exp(-0.5 * np.dot(np.dot(x - mu, iEaa), x - mu)) / fEaa

        # note that for extreme values this differs from fgmm, but we match R!
        rospy.logdebug("x : %s mu : %s precision : %s  answer : %s" % (str(x), str(mu), str(precision), str(answer)))

        return answer


class GMR(object):

    def __init__(self, filename):

        fp = open(filename)
        raw_data = [] # store for parameters which will populate mu and sigma structures
        for (i,line) in enumerate(fp):

            if (i == 0):
                self.dim = int(line)
            elif (i == 1):
                self.ncomp = int(line)
            elif (i == 3):
                self.offset = npa(line.split(),dtype='double')
            elif (i == 5):
                self.priors = npa(line.split(),dtype='double')
            elif (i > 5):
                # store all parameters in the raw data array -- use
                # this to populate mu and sigma objects -- otherwise
                # indexing gets too incomprehensible
                raw_data.extend([float(x) for x in line.split()])

        # now populate self.mus and self.sigmas based on the raw parameters
        size = 2 * self.dim
        self.mus = npa(raw_data[:self.ncomp*size]).reshape((size,self.ncomp))
        del raw_data[:self.ncomp * size]

        self.sigmas = []
        for i in range(self.ncomp):
            self.sigmas.append(npa(raw_data[:size**2]).reshape((size,size)))
            del raw_data[:size**2]
        self.sigmas = npa(self.sigmas)

        # do a bunch of precomputations
        self.subsigmas = []
        for i in range(self.ncomp):
            sigma = self.sigmas[i]
            d = self.dim

            Eaa = sigma[:d,:d]
            Ebb = sigma[d:,d:]
            Eab = sigma[:d,d:]
            Eba = sigma[d:,:d]

            # we are going to be computing b | a conditional probabilities
            # b : dx
            # a : x

            iEaa = la.inv(Eaa)
            det = la.det(Eaa)
            Ebca = Ebb - np.dot(np.dot(Eba, iEaa), Eab) # from Bishop pg. 87
            self.subsigmas.append({'E' : sigma, 'det' : det,
                                   'Eaa' : Eaa, 'Ebb' : Ebb,
                                   'Eab' : Eab, 'Eba' : Eba,
                                   'iEaa' : iEaa, 'Ebca' : Ebca})

    def compute_conditional_priors(self,x):
        self.cpriors = np.zeros(self.ncomp)

        for i in range(self.ncomp):
            mua = self.mus[:self.dim,i]
            precision = self.subsigmas[i]['iEaa']
            det = self.subsigmas[i]['det']
            self.cpriors[i] = self.priors[i] * self.normal(x,mua,det,precision)

        # normalize cpriors
        self.cpriors = self.cpriors / np.sum(self.cpriors)

    def compute_conditional_means(self,x):

        self.cmeans = []

        for i in range(self.ncomp):
            Ebb = self.subsigmas[i]['Ebb']
            iEaa = self.subsigmas[i]['iEaa']
            Eba = self.subsigmas[i]['Eba']
            mua = self.mus[:self.dim,i]
            mub = self.mus[self.dim:,i]

            # from Bishop pg. 87
            mubca = mub + np.dot(np.dot(Eba, iEaa), (x - mua))
            self.cmeans.append(mubca)

    def normal(self, x, mu, det, precision):
        """ Return the normal density at a point x for component i. """
        D = len(x)
        div = (2.0 * np.pi)**(D / 2.0) * (det)**(0.5)
        answer = np.exp(-0.5 * np.dot(np.dot(x - mu, precision), x - mu)) / div

        # note that for extreme values this differs from fgmm, but we match R!
        rospy.logdebug("x : %s mu : %s precision : %s  answer : %s" % (str(x), str(mu), str(precision), str(answer)))

        return answer

    def regression(self, x):
        self.compute_conditional_means(x)
        self.compute_conditional_priors(x)
        return np.dot(self.cpriors, self.cmeans)


def load_model(req):
    global gmr
    rospy.loginfo("Loading model %s" % req.filename)
    gmr = GMR(req.filename)
    return DSLoadResponse()

def ds_server(req):
    global gmr
    if gmr == None:
        rospy.loginfo("No model loading!")
        return DSSrvResponse()
    else:
        nx = (npa(req.x) - gmr.offset[:gmr.dim]) * 1000.0
        dx = gmr.regression(nx) / 1000.0
        rospy.logdebug("x : %s dx : %s" % (str(req.x), str(dx)))
        return DSSrvResponse(dx)

def init():
    rospy.init_node('gmr')
    rospy.Service('load_model', DSLoad, load_model)
    rospy.Service('ds_server', DSSrv, ds_server)
    rospy.loginfo('ds_node ready.')
    rospy.spin()

if __name__ == '__main__':

    init()
