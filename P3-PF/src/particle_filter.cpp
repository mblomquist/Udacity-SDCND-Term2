/*
* particle_filter.cpp
*
*  Created on: Dec 12, 2016
*      Author: Tiffany Huang
*/

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#define _USE_MATH_DEFINES
#include <math.h> 

#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
      // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
      //   x, y, theta and their uncertainties from GPS) and all weights to 1. 
      // Add random Gaussian noise to each particle.
      // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

      // Set number of particles
      num_particles = 100;

      // Initialize weights
      weights.resize(num_particles);

      // Initialize random gen
      default_random_engine gen;

      // Initialize distributions
      normal_distribution<double> dist_x(x, std[0]);
      normal_distribution<double> dist_y(y, std[1]);
      normal_distribution<double> dist_theta(theta, std[2]);

      // Reserve Space for Number of Particles
      particles.reserve(num_particles);

      // Place Particles into Particle Vector
      for (int i = 0; i < num_particles; i++) {
            Particle p_init = { 0, dist_x(gen), dist_y(gen), dist_theta(gen), 1 };
            particles.push_back(p_init);
            weights[i] = 1.0;
      }

      // Mark Initialized
      is_initialized = true;

      return;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
      // TODO: Add measurements to each particle and add random Gaussian noise.
      // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
      //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
      //  http://www.cplusplus.com/reference/random/default_random_engine/

      // Initialize random gen
      default_random_engine gen;

      // Create normal distributions
      normal_distribution<double> x_dist(0.0, std_pos[0]), y_dist(0.0, std_pos[1]), theta_dist(0.0, std_pos[2]);

      // Run particle prediction loop
      for (Particle &p : particles) {

            // Check for divition by zero
            if (fabs(yaw_rate) > .001) {
                  p.x += (velocity / yaw_rate) * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
                  p.y += (velocity / yaw_rate) * (cos(p.theta) - cos(p.theta + yaw_rate*delta_t));
                  p.theta += yaw_rate * delta_t;
            }
            else {
                  p.x += velocity * cos(p.theta) * delta_t;
                  p.y += velocity * sin(p.theta) * delta_t;
                  p.theta += yaw_rate * delta_t;
            }

            // Update probabilities
            p.x += x_dist(gen);
            p.y += y_dist(gen);
            p.theta += theta_dist(gen);

      }

      return;

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
      // TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
      //   observed measurement to this particular landmark.
      // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
      //   implement this method and use it as a helper during the updateWeights phase.

      // I decided not to use this helper function because it was easier to optimize the updateWeights routine
      //   alone.

      return;

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
      std::vector<LandmarkObs> observations, Map map_landmarks) {
      // TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
      //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
      // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
      //   according to the MAP'S coordinate system. You will need to transform between the two systems.
      //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
      //   The following is a good resource for the theory:
      //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
      //   and the following is a good resource for the actual equation to implement (look at equation 
      //   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
      //   for the fact that the map's y-axis actually points downwards.)
      //   http://planning.cs.uiuc.edu/node99.html

      // Ideas for computation flow and speed imrpovement methods came from
      //   NikolasEnt @ https://github.com/NikolasEnt
      
      // Create constants to reduce repeat computation
      const double std_lm_x = std_landmark[0] * std_landmark[0];
      const double std_lm_y = std_landmark[1] * std_landmark[1];
      const double w_const = 2 * M_PI * std_landmark[0] * std_landmark[1];

      // Set initial values for computation
      double dx = 0.0;
      double dy = 0.0;
      double w_sum_temp = 0.0;

      // Create loop to update weights
      for (int i = 0; i < num_particles; i++) {

            // Create variable weight to improve computation
            double w_temp = 0.0;

            // Create const values to reduce computation
            const double sin_theta = sin(particles[i].theta);
            const double cos_theta = cos(particles[i].theta);

            // Start the observations loop
            for (int j = 0; j < observations.size(); j++) {

                  // Observation measurement transformations
                  LandmarkObs observation;
                  observation.x = particles[i].x + (observations[j].x * cos_theta) - (observations[j].y * sin_theta);
                  observation.y = particles[i].y + (observations[j].x * sin_theta) + (observations[j].y * cos_theta);

                  // Set in_range value to false
                  bool in_range = false;

                  // Initialize variable (holder) for nearest landmark
                  Map::single_landmark_s nearest_lm;

                  // Set initial threshold for distance to landmark -- Start big.
                  double nearest_dist = 1e9;

                  // Run nearest neightbor search
                  for (int k = 0; k < map_landmarks.landmark_list.size(); k++) {

                        // Set temp landmark
                        Map::single_landmark_s temp_lm = map_landmarks.landmark_list[k];

                        // Calulate distance between landmark and oberservation
                        double distance = dist(temp_lm.x_f, temp_lm.y_f, observation.x, observation.y);

                        // Check for shorter distance
                        if (distance < nearest_dist) {

                              // Set new threshold for nearest distance
                              nearest_dist = distance;

                              // Set new condition for temp landmark
                              nearest_lm = temp_lm;

                              // Check if distance is within sensor range
                              if (distance < sensor_range) {
                                    in_range = true;
                              }
                        }
                  }

                  // Update weight variable if distance < sensor_range is found
                  if (in_range) {

                        // Calculate distance from observation to nearest landmark
                        dx = observation.x - nearest_lm.x_f;
                        dy = observation.y - nearest_lm.y_f;

                        // Add to temp weight
                        w_temp += dx * dx / std_lm_x + dy * dy / std_lm_y;
                  }

                  // Set weight high if distance is not found
                  // Note: This idea came from NikolasEnt @ https://github.com/NikolasEnt
                  // >> When w_temp is high, the weighted probaility goes to 0. 
                  else {
                        w_temp += 100.0;
                  }
            }


            // Calculate particle weight after loop to reduce computational time.
            particles[i].weight = exp(-0.5*w_temp);
            w_sum_temp += particles[i].weight;
      }

      // Normalize weights to sum to 1
      for (int i = 0; i < num_particles; i++) {
            particles[i].weight /= w_const * w_sum_temp;
            weights[i] = particles[i].weight;
      }

      return;

}

void ParticleFilter::resample() {
      // TODO: Resample particles with replacement with probability proportional to their weight. 
      // NOTE: You may find std::discrete_distribution helpful here.
      //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

      // Create a random device
      random_device r_weights;
      mt19937 g_weights(r_weights());

      // Creates a discrete distribution for weight.
      discrete_distribution<int> d_weights(weights.begin(), weights.end());
      vector<Particle> resampled_particles;

      // Resample particles
      for (int i = 0; i<num_particles; i++) {

            Particle particles_i = particles[d_weights(g_weights)];
            resampled_particles.push_back(particles_i);
      }

      // Set particles to resampled values
      particles = resampled_particles;

      return;
}

void ParticleFilter::write(std::string filename) {
      // You don't need to modify this file.
      std::ofstream dataFile;
      dataFile.open(filename, std::ios::app);
      for (int i = 0; i < num_particles; ++i) {
            dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
      }
      dataFile.close();
}
