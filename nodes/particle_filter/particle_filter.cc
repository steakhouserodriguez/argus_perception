/*
 * particle_filter.cpp
 *
 * Particle filter for color-based object tracking in OpenCV
 *
 * Based loosely on the BallBot particle filter
 * https://github.com/preetnum/ballbot
 *
 *  Created on: Sep 08, 2012
 *      Author: Ryan Julian
 *  
 */
/* C Standard Library */
#include <math.h>

/* OpenCV */
#include <opencv2/opencv.hpp>

/* Project */
#include "particle_filter.h"
#include <typeinfo>

using namespace cv;
using namespace std;

/* Globals */
cv::RNG rng(time(NULL));

/* ParticleFilter */
ParticleFilter::ParticleFilter(unsigned int n) { }
ParticleFilter::ParticleFilter(unsigned int n, 
                               vector<Range> bounds,
                               TransitionModel transition_model,
                               EmissionModel emission_model)
    : num_particles_(n),
      bounds_(bounds),
      transition_model_(transition_model),
      emission_model_(emission_model)
{
  particles_ = Mat::zeros(num_particles_, bounds_.size(), DataType<short>::type);
  weights_ = Mat::ones(num_particles_, 1, DataType<double>::type);
  InitializeUniformly();
}

ParticleFilter::ParticleFilter(Mat initial_particles,
                               vector<Range> bounds,
                               TransitionModel transition_model,
                               EmissionModel emission_model)
    : num_particles_(initial_particles.rows),
      bounds_(bounds),
      transition_model_(transition_model),
      emission_model_(emission_model)
{
  weights_ = Mat::ones(num_particles_, 1, DataType<double>::type);
  Normalize();
}

void ParticleFilter::InitializeUniformly() {
  // Initialize each dimension uniform randomly within its bounds
  Mat col;
  for(unsigned int i = 0; i < bounds_.size(); i++) {
    col = particles_.col(i);
    randu(col, Scalar(bounds_[i].start), Scalar(bounds_[i].end));
  }
  Normalize();
}

// TODO: better emission model
void ParticleFilter::Observe(Mat frame) {
    
  // Weight ~ inverse quadratic color distance
  Mat_<short> p;
  Vec3b color, em_color;
  for(unsigned int i = 0; i < num_particles_; i++) {
    p = particles_.row(i);
    color = frame.at<Vec3b>(p(0), p(1));
    em_color = emission_model_.at<Vec3b>(p(0), p(1));
    //weights_.at<double>(i) = 1/(1 + pow(norm(color, emission_model_), 2));
    weights_.at<double>(i) = 1 + pow(norm(color, em_color), 2);
    // TODO: calculate cumsum of weights, to speed resampling
  }

  // Normalize
  Normalize();
  
  // Resample if particles degenerate
  // TODO: better resampling condition
  if(1/pow(norm(weights_), 2) < num_particles_/2) {
    Resample();
  }
}

void ParticleFilter::ElapseTime() {
  Mat g, col_gaussian, col_uniform;
  
  // Gaussian random walk with epsilon uniform randomly distributed
  int num_uniform = (int) (transition_model_.epsilon * (double) num_particles_);
  
  for(unsigned int i = 0; i < bounds_.size(); i++) {
    // Get columns for gaussian and uniform sets
    col_gaussian = particles_(Range(num_uniform, num_particles_-1),Range(i,i+1));
    col_uniform = particles_(Range(0, num_uniform), Range(i,i+1));
  
    // Gaussian
    g = Mat(col_gaussian.size(), col_gaussian.type());
    randn(g, Scalar(transition_model_.mu), Scalar(transition_model_.sigma));
    //randu(g, Scalar(-transition_model_.mu), Scalar(transition_model_.mu));
    col_gaussian += g;
    
    // Threshold particles outside the bounds
    min(col_gaussian, bounds_[i].end, col_gaussian);
    max(col_gaussian, bounds_[i].start, col_gaussian);
    
    // Uniform
    randu(col_uniform, Scalar(bounds_[i].start), Scalar(bounds_[i].end));
  }
}

State ParticleFilter::Estimate() {
  Mat float_particles;
  particles_.convertTo(float_particles, weights_.type());
  Mat_<double> p = float_particles.t()*weights_;
  return State(p(1), p(0));
}

void ParticleFilter::Resample() {
  // Stochastic Universal Sampling
  // J.E. Baker. "Reducing bias an inefficiency in selection algorithms.", 1987
  Mat cumsum, new_row;
  integral(weights_, cumsum);                         // Find the cumulative sum of the weights
  cumsum = cumsum.col(1);                             // Workaround: integral always returns a 2D array
  double stride = 1./num_particles_;                  // Walk the array in equal strides of total weight
  
  int old_idx = 0;                                    // Index in old population
  int new_idx = 0;                                    // Index in new population
  for(double marker = stride * rng.uniform(0., 1.);   // Start the walk at a random point inside the first stride
      marker < 1; marker += stride) {                 // Walk in equal strides thereafter until reaching the end
    while(marker > cumsum.at<double>(old_idx)) {      // Find the particle the marker points to
      old_idx++;
    }
    new_row = particles_.row(new_idx);
    particles_.row(old_idx-1).copyTo(new_row);        // Add the particle under the marker to the new population
    new_idx++;
  }
}

void ParticleFilter::Normalize() {
  normalize(weights_, weights_, 1, 0, NORM_L1);
}

State ParticleFilter::Draw(Mat frame) {
  State estimate = Estimate();
  circle(frame, estimate, 10, Scalar(255, 0, 0), 2); // Draw position estimate
  Mat_<short> p;
  for(unsigned int i = 0; i < num_particles_; i++) {      // Draw particles
    p = particles_.row(i);
    frame.at<Vec3b>(p(0),p(1))[0] = 0;
    frame.at<Vec3b>(p(0),p(1))[1] = 0;
    frame.at<Vec3b>(p(0),p(1))[2] = 255;
  }
  return estimate;
}
