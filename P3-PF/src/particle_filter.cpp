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

      default_random_engine gen;

      normal_distribution<double>
            dist_x(x, std[0]),
            dist_y(y, std[1]),
            dist_theta(theta, std[2]);

      // Reserve Space for Number of Particles
      particles.reserve(num_particles);

      // Place Particles into Particle Vector
      for (int i = 0; i < num_particles; i++) {
            Particle p_init = { 0, dist_x(gen), dist_y(gen), dist_theta(gen), 1 };
            particles.push_back(p_init);
      }

      // Mark Initialized
      is_initialized = true;
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

      weights.clear();
      weights.reserve(num_particles);

      double std_x = std_landmark[0];
      double std_y = std_landmark[1];

      double c = 0.5 / (M_PI*std_x*std_y);

      if (std_x < 0.001 || std_y < 0.001) {
            c = 0.000001;
      }

      for (Particle &p : particles)
      {
            std::vector<LandmarkObs> landmarks_in_sensor_range;
            std::vector<LandmarkObs> observations_in_map_coords;
            observations_in_map_coords.reserve(observations.size());

            // transform observations into map coordinates using the perspective of the current particle
            for (const LandmarkObs &obs : observations)
            {
                  LandmarkObs transformed_obs;

                  transformed_obs.id = 0;
                  transformed_obs.x = p.x + obs.x * cos(p.theta) - obs.y * sin(p.theta);
                  transformed_obs.y = p.y + obs.x * sin(p.theta) + obs.y * cos(p.theta);

                  observations_in_map_coords.push_back(transformed_obs);
            }

            // find landmarks in the map which are within sensor_range of the current particle
            for (const Map::single_landmark_s &lm : map_landmarks.landmark_list)
            {
                  if (dist(p.x, p.y, lm.x_f, lm.y_f) <= sensor_range)
                  {
                        LandmarkObs lm_tmp = { lm.id_i, lm.x_f, lm.y_f };
                        landmarks_in_sensor_range.push_back(lm_tmp);
                  }
            }

            // associate in-range landmarks with transformed sensor readings
            dataAssociation(observations_in_map_coords, landmarks_in_sensor_range);

            // calculate the weight of the particle
            double prob = 1.0;

            for (const LandmarkObs &lm : landmarks_in_sensor_range)
            {
                  const LandmarkObs &closest_obs = observations_in_map_coords[lm.id];

                  const double x_diff = pow((closest_obs.x - lm.x) / std_x, 2),
                        y_diff = pow((closest_obs.y - lm.y) / std_y, 2);

                  prob *= c * exp(-0.5 * (x_diff + y_diff));

                  // Don't let a particle have zero probability of being chosen
                  if (prob < 1e-4)
                        break;
            }

            // store the probability of this particle being real in the weight member and the weights_ vector.
            p.weight = prob;
            weights.push_back(prob);
      } // end particle loop

      return;
            
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

          // Create random index generator where the probability of each particle index to be selected
    // is equivalent to its weight.
    std::default_random_engine gen;
    std::discrete_distribution<int> d(weights.begin(), weights.end());

    // temporary list to store re-sampled particles
    std::vector<Particle> resampled_particles;
    resampled_particles.reserve( num_particles );

    // generate a random index and append the corresponding particle to the re-sampling list
    for (int i = 0; i < num_particles; ++i)
        resampled_particles.push_back( particles[d(gen)] );

    // replace the old particles with the re-sampled particles
    particles = resampled_particles;
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
