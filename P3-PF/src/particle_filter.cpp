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
      num_particles = 100;
	  weights.resize(num_particles);

      default_random_engine gen;

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

      default_random_engine gen;
      normal_distribution<double>
            x_dist(0.0, std_pos[0]),
            y_dist(0.0, std_pos[1]),
            theta_dist(0.0, std_pos[2]);

      for (Particle &p : particles) {

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

      for (LandmarkObs &obs : observations) {

            double min_distance = 0.000001;

            for (int i = 0; i < predicted.size(); i++) {

                  if (predicted[i].id == 0) {

                        const double d = dist(obs.x, obs.y, predicted[i].x, predicted[i].y);

                        if (d < min_distance) {
                              obs.id = i;
                              min_distance = d;
                        }
                  }
            }
      }

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
    
	const double sigma_xx = std_landmark[0] * std_landmark[0];
	const double sigma_yy = std_landmark[1] * std_landmark[1];
	const double k = 2 * M_PI * std_landmark[0] * std_landmark[1];
	double dx = 0.0;
	double dy = 0.0;
	double sum_w = 0.0; // Sum of weights for future weights normalization
	for (int i = 0; i < num_particles; i++) {

		  double weight_no_exp = 0.0;
		  const double sin_theta = sin(particles[i].theta);
		  const double cos_theta = cos(particles[i].theta);
		  for (int j = 0; j < observations.size(); j++) {

				  // Observation measurement transformations
				  LandmarkObs observation;
				  observation.id = observations[j].id;
				  observation.x = particles[i].x + (observations[j].x * cos_theta) - (observations[j].y * sin_theta);
				  observation.y = particles[i].y + (observations[j].x * sin_theta) + (observations[j].y * cos_theta);

				  // Unefficient way for observation asossiation to landmarks. It can be improved.
				  bool in_range = false;
				  Map::single_landmark_s nearest_lm;
				  double nearest_dist = 10000000.0; // A big number
				  for (int k = 0; k < map_landmarks.landmark_list.size(); k++) {
						Map::single_landmark_s cond_lm = map_landmarks.landmark_list[k];
						double distance = dist(cond_lm.x_f, cond_lm.y_f, observation.x, observation.y);  // Calculate the Euclidean distance between two 2D points
						if (distance < nearest_dist) {
							  nearest_dist = distance;
							  nearest_lm = cond_lm;
							  if (distance < sensor_range) {
									in_range = true;
							  }
						}
				  }

				  if (in_range) {
						dx = observation.x - nearest_lm.x_f;
						dy = observation.y - nearest_lm.y_f;
						weight_no_exp += dx * dx / sigma_xx + dy * dy / sigma_yy;
				  }

				  else {
						weight_no_exp += 100; // approx = 0 after exp()
				  }
			}
			particles[i].weight = exp(-0.5*weight_no_exp); // calculate exp() after main computation in order to optimize the code
			sum_w += particles[i].weight;
	  }
	  // Weights normalization to sum(weights)=1
	  for (int i = 0; i < num_particles; i++) {
			particles[i].weight /= sum_w * k;
			weights[i] = particles[i].weight;
	  }

      return;
            
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	std::random_device rd_wts;
	std::mt19937 generator_wts(rd_wts());

	// Creates a discrete distribution for weight.
	std::discrete_distribution<int> distribution_wts(weights.begin(), weights.end());
	std::vector<Particle> resampled_particles;

	// Resample
	for (int i = 0; i<num_particles; i++) {
		  Particle particles_i = particles[distribution_wts(generator_wts)];
		  resampled_particles.push_back(particles_i);
	}
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
