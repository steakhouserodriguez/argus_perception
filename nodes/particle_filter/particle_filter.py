# Blob-Tracking Particle Filter
# Author: Ryan C. Julian
#
# Based loosely on the BallBot particle filter and SciPy particle filter
# https://github.com/radicalnerd/ballbot/
# http://www.scipy.org/Cookbook/ParticleFilter

from numpy import *
from numpy.random import *

class ParticleFilter:
    # TODO: Generic transition and emission models

    def __init__(self, pos, stepsize, bounds, num_particles=1000):
        self.num_particles = num_particles

        # list of (x,y) particle positions.
        self.particles = ones((num_particles,2), int) * pos     # Initial position

        # weights
        self.w = ones(num_particles)/num_particles
        self.ss = stepsize
        self.bounds = bounds

    def observe(self, observation):
        # TODO: better emission model

        # Weight = frame difference, directly from pipeline
        self.w = observation[tuple(self.particles.T)]

        # Normalize w
        self.w /= sum(self.w)

        # TODO: better resampling condition
        if 1./sum(self.w**2) < self.num_particles/10.:           # Resample if particles degenerate
            self._resample()

    def elapse_time(self):
        # TODO: better transition model
        # TODO: random particles
        self.particles += uniform(-self.ss, self.ss, self.particles.shape)                  # Uniform step motion model
        self.particles = self.particles.clip(zeros(2), array(self.bounds)-1).astype(int)    # Clip out-of-bounds particles

    def _resample(self):
        indices = []
        C = [0.] + [sum(self.w[:i+1]) for i in range(self.num_particles)]
        u0, j = random(), 0
        for u in [(u0+i)/self.num_particles for i in range(self.num_particles)]:
            while u > C[j]:
                j+=1
            indices.append(j-1)
        self.particles = self.particles[indices,:]

    def get_beliefs(self):
        return self.particles

    def get_estimate(self):
        return sum(self.particles.T*self.w, axis=1).astype(uint16)

    def get_weights(self):
        return self.w

    def get_state(self):
        return self.get_estimate(), self.get_beliefs(), self.get_weights()




