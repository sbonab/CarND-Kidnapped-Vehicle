/**
 * particle_filter.cpp
 *
 * Created on: Feb 24, 2021
 * Author: Saeed Bonab
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double sigma[]) {

  num_particles = 100;  // Set the number of particles

  // Initialize all particles to 
  //  first position (based on estimates of x, y, theta and their uncertainties
  //   from GPS) and all weights to 1.
  // Add random Gaussian noise to each particle
  std::default_random_engine gen; // random number generator engine
  std::normal_distribution<double> x_dist(x, sigma[0]);
  std::normal_distribution<double> y_dist(y, sigma[1]);
  std::normal_distribution<double> theta_dist(theta, sigma[2]);
  for (int i = 0; i < num_particles; ++i){
      Particle particle;
      // assigning random numbers to x,y,theta
      particle.x = x_dist(gen);;
      particle.y = y_dist(gen);
      particle.theta = theta_dist(gen);
      // initializing weight = 1
      particle.weight = 1.0;
      // appending to particles
      particles.push_back(particle);
      // append to the vector of weights
      weights.push_back(particle.weight);
  }

  // Flag, if filter is initialized
  is_initialized = true;
}

void ParticleFilter::revive(double sigma[]){
  // number of the childs of each particle
  int n = 10;
  
  std::default_random_engine gen; // random number generator engine
  std::vector<Particle> new_particles; // vector of new particles
  for (int k = 0; k < particles.size(); ++k){
    auto particle = particles[k];
    // Add random Gaussian noise to each particle child
    std::normal_distribution<double> x_dist(particle.x, sigma[0]);
    std::normal_distribution<double> y_dist(particle.y, sigma[1]);
    std::normal_distribution<double> theta_dist(particle.theta, sigma[2]);
    for (int i = 0; i < n; ++i){
      auto new_particle = particle;
      new_particle.x = x_dist(gen);;
      new_particle.y = y_dist(gen);
      new_particle.theta = theta_dist(gen);

      // appending to the vector of new particles
      new_particles.push_back(new_particle);
    }
  }
  // appending the new particles to the end of the old ones
  particles.insert(particles.end(), new_particles.begin(), new_particles.end());
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
 
  // avoid division by zero
  double eps = 0.00000001; 
  if (yaw_rate < eps && yaw_rate >= 0){
      yaw_rate = eps;
  } else if (yaw_rate > -eps && yaw_rate < 0){
      yaw_rate = -eps;
  }
  
  std::default_random_engine gen; // random number generator engine
  
  for ( int i = 0; i < particles.size(); ++i ){
      auto particle = particles[i];
      // updating x
      particle.x += (velocity / yaw_rate) * ( sin(particle.theta + yaw_rate*delta_t) - sin(particle.theta) );
      std::normal_distribution<double> x_dist(particle.x, std_pos[0]);
      particle.x = x_dist(gen);
      // updating y
      particle.y += (velocity / yaw_rate) * ( -cos(particle.theta + yaw_rate*delta_t) + cos(particle.theta) );
      std::normal_distribution<double> y_dist(particle.y, std_pos[1]);
      particle.y = y_dist(gen);
      // updating theta
      particle.theta += yaw_rate * delta_t;
      std::normal_distribution<double> theta_dist(particle.theta, std_pos[2]);
      particle.theta = theta_dist(gen);

      // assigning to particles
      particles[i] = particle;
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /*
   *   Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   */
  // looping over observations
  for (int i = 0; i < observations.size(); ++i){
      auto obs = observations[i];
      // index of the associating landmark in the map
      int idx = 0;
      // very large initial distance
      double d = 100000000.0;
      for (int j = 0; j < predicted.size(); ++j){
        // calculate the distance
        double new_d = dist(obs.x, obs.y, predicted[j].x, predicted[j].y);
        // check if the distance is smaller than the running minimum
        if (new_d < d){
            d = new_d;
            idx = predicted[j].id;
        }
      }
      // updating the index in the observation
      observations[i].id = idx;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   *   Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. 
   */
  // looping over the particles
  for (int k = 0; k < particles.size(); ++k){
    auto particle = particles[k];
    // transforming the observations from car coordinate to the world coordinate
    vector<LandmarkObs> transformed_obs;
    for (int i = 0; i < observations.size(); ++i){
      // particle info
      double x_part = particle.x;
      double y_part = particle.y;
      double theta = particle.theta;
      // observations
      double x_obs = observations[i].x;
      double y_obs = observations[i].y;
      // homogenous transformation equations
      LandmarkObs obs_m; // observation in map coordinates
      obs_m.x = x_part + ( x_obs * cos(theta) ) - ( y_obs * sin(theta) );
      obs_m.y = y_part + ( x_obs * sin(theta) ) + ( y_obs * cos(theta) );
      // adding it to the vector of transormed observation
      transformed_obs.push_back(obs_m);
    }
    
    // the predicted observations
    vector<LandmarkObs> predicted_obs;
    for (int i = 0; i < map_landmarks.landmark_list.size(); ++i){
      // all of the following values are in map coordinates
      double x_land = map_landmarks.landmark_list[i].x_f;
      double y_land = map_landmarks.landmark_list[i].y_f;
      double x_part = particle.x;
      double y_part = particle.y;
      // finding the distance and comparing it to range
      if (dist(x_part, y_part, x_land, y_land) < sensor_range){
        LandmarkObs obs;
        obs.x = x_land;
        obs.y = y_land;
        obs.id = map_landmarks.landmark_list[i].id_i;
        predicted_obs.push_back(obs);
      }
    }
    // finidng data associtation
    dataAssociation(predicted_obs, transformed_obs);
    // updating the weight
    particle.weight = 1.0;
   for (int i = 0; i < transformed_obs.size(); ++i){
     for (int j = 0; j < predicted_obs.size(); ++j){
       if (transformed_obs[i].id == predicted_obs[j].id){
         particle.weight *= multiv_prob(std_landmark[0], std_landmark[1], transformed_obs[i].x, transformed_obs[i].y, predicted_obs[j].x, predicted_obs[j].y);
       }
     }
   } 
     // setting associations
    std::vector<int> associations;
    std::vector<double> sense_x;
    std::vector<double> sense_y;
    for (int i = 0; i < transformed_obs.size(); ++i){
      associations.push_back(transformed_obs[i].id);
      sense_x.push_back(transformed_obs[i].x);
      sense_y.push_back(transformed_obs[i].y);
      SetAssociations(particle, associations, sense_x, sense_y);
    }
    // assigning back the particle
    particles[k] = particle;
  }
  // updating the weight vector
  weights.clear();
  for (int i = 0; i < particles.size(); ++i){
    weights.push_back(particles[i].weight);
  }
  // normalizing the weight vector
  normalize(weights);
  // updating the weight of particles
  for (int i = 0; i < particles.size(); ++i){
    particles[i].weight = weights[i];
  }
}

void ParticleFilter::resample() {
  /**
   *  Resample particles with replacement with probability proportional 
   *  to their weight. 
   */
  // setting up random engine with weighted distribution
  std::random_device rd;
  std::mt19937 gen(rd());
  std::discrete_distribution<int> d(weights.begin(), weights.end());
  // vector that will hold the random indices
  std::vector<int> indices;
  for (int i = 0; i < particles.size(); ++i){
    indices.push_back(d(gen));
  }
  // removing dupliactes from indices
  std::set<int> indices_set(indices.begin(), indices.end());
  indices.assign(indices_set.begin(), indices_set.end());
  // updating particles vector
  std::vector<Particle> new_particles;
  for (int i = 0; i < indices.size(); ++i){
    new_particles.push_back(particles[indices[i]]);
  }
  particles = new_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}