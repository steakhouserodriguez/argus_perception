/*
 * particle_filter.hpp
 *
 * Particle filter for color-based object tracking in OpenCV
 *
 * Based loosely on the BallBot particle filter.
 * https://github.com/preetnum/ballbot
 *
 *  Created on: Sep 08, 2012
 *      Author: Ryan Julian
 *  
 */
#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

/* CPP Standard Library */
#include <vector>

/* OpenCV */
#include <opencv2/opencv.hpp>

typedef cv::Point State;
//typedef cv::Vec3b EmissionModel;
typedef cv::Mat EmissionModel;

struct TransitionModel {
  double mu;
  double sigma;
  double epsilon;
};

class ParticleFilter {
 public:
  /**
   * Constructors
   */
  ParticleFilter();
  ParticleFilter(unsigned int n);
  ParticleFilter(unsigned int n, std::vector<cv::Range> bounds,
                 TransitionModel transition_model,
                 EmissionModel emission_model);
  ParticleFilter(cv::Mat initial_particles, std::vector<cv::Range> bounds,
                 TransitionModel transition_model,
                 EmissionModel emission_model);
  
  /**
   * Initialization
   */
  void InitializeUniformly();
  
  /**
   * Observe a frame and reweight particles according the emission
   * model
   */
  void Observe(cv::Mat frame);
  
  /**
   * Elapse time according to the transition model
   */
  void ElapseTime();
  
  /**
   * Estimate state
   */
  State Estimate();

  /**
   * Draw
   */
  State Draw(cv::Mat frame);

  /**
   * Accessors
   */
  unsigned int num_particles() { return num_particles_; }
  std::vector<cv::Range> bounds() { return bounds_; }
  cv::Mat particles() { return particles_; }
  cv::Mat weights() { return weights_; }
 
 private:
  unsigned int num_particles_;
  std::vector<cv::Range> bounds_;
  TransitionModel transition_model_;
  EmissionModel emission_model_;
  cv::Mat particles_;
  cv::Mat weights_;

  /**
   * Resampling
   */
  void Resample();

  /**
   * Normalization
   */
  void Normalize();
};
#endif
