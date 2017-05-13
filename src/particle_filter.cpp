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

#include "particle_filter.h"

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	/*******************************************************************************************************
	* Set the number of particles. Initialize all particles to first position (based on estimates of 
	* x, y, theta and their uncertainties from GPS) and all weights to 1. 
	* Add random Gaussian noise to each particle.
	* NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	********************************************************************************************************/

	// No. particles
	num_particles = 20;
	
	std::normal_distribution<double> dist_x(0, std[0]);
	std::normal_distribution<double> dist_y(0, std[1]);
	std::normal_distribution<double> dist_theta(0, std[2]);
	std::default_random_engine gen;

	for (int i = 0; i < particles.size(); ++i) {
		// Initial particle position and noise
		Particle p;
		p.id = 1;
		p.x = x + dist_x(gen);
		p.y = y + dist_y(gen);
		p.theta = theta + dist_theta(gen);

		// Initial weight of 1
		p.weight = 1;

		weights.push_back(p.weight);
		particles.push_back(p);
	}

	is_initialized = true;
	
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	/**********************************************************************************************************
	* Add measurements to each particle and add random Gaussian noise.
	* NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	* http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	* http://www.cplusplus.com/reference/random/default_random_engine/
	************************************************************************************************************/

	// Random Gausian noise distribution
	std::normal_distribution<double> noise_x(0, std_pos[0]);
	std::normal_distribution<double> noise_y(0, std_pos[1]);
	std::normal_distribution<double> noise_theta(0, std_pos[2]);
	std::default_random_engine gen;

	for (unsigned int i = 0; i < particles.size(); ++i) {
		// Initialize predictions
		double x_pred;
		double y_pred;
		double theta_pred;

		// Update prediction state
		if (yaw_rate < 0.001) {
			x_pred = particles[i].x + velocity * cos(particles[i].theta) * delta_t;
			y_pred = particles[i].y + velocity * sin(particles[i].theta) * delta_t;
			theta_pred = particles[i].theta + yaw_rate * delta_t;
		} else {
			x_pred = particles[i].x + (velocity / yaw_rate) * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			y_pred = particles[i].y + (velocity / yaw_rate) * (-cos(particles[i].theta + yaw_rate * delta_t) + cos(particles[i].theta));
			theta_pred = particles[i].theta + yaw_rate * delta_t;
		}

		// Add noise
		particles[i].x = x_pred + noise_x(gen);
		particles[i].y = y_pred + noise_y(gen);
		particles[i].theta = theta_pred + noise_theta(gen);
	}
}


void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	/****************************************************************************************************************
	* Find the predicted measurement that is closest to each observed measurement and assign the observed
	* measurement to this particular landmark.
	* NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	* implement this method and use it as a helper during the updateWeights phase.
	*****************************************************************************************************************/

	// Initialize measurement predictions
	for (unsigned int i = 0; i < observations.size(); ++i) {
		double cur_dist = 1e6;
		int landmark = -1;

		for (unsigned int j = 0; j < predicted.size(); ++j) {
			double s_dist = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);

			if (s_dist < cur_dist) {
				cur_dist = s_dist;
				landmark = j;
			}
		}
		
		observations[i].id = predicted[landmark].id;

	} 
}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	/********************************************************************************************************
	* Update the weights of each particle using a mult-variate Gaussian distribution. You can read more 
	* about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	* NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	* according to the MAP'S coordinate system. You will need to transform between the two systems. Keep in
	* mind that this transformation requires both rotation AND translation (but no scaling).
	* The following is a good resource for the theory:
	* https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	* and the following is a good resource for the actual equation to implement (look at equation 
	* 3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	* for the fact that the map's y-axis actually points downwards.)
	* http://planning.cs.uiuc.edu/node99.html
	*********************************************************************************************************/

	// Initialize weights
	weights.clear();

	for (unsigned int i = 0; i < particles.size(); ++i) {
		// convert observation into map
		std::vector<LandmarkObs> map_obs;
		for (unsigned int j = 0; j < observations.size(); ++j) {
			if (dist(observations[j].x, observations[j].y, 0, 0) <= sensor_range) {
				LandmarkObs obs;
				obs.x = particles[i].x + observations[j].x * cos(particles[i].theta) - observations[j].y * sin(particles[i].theta);
				obs.y = particles[i].y + observations[j].y * sin(particles[i].theta) + observations[j].y * cos(particles[i].theta);
				obs.id = -1;
				map_obs.push_back(obs);
			}
		}
		
		// map landmarks
		std::vector<LandmarkObs> landmarks;
		for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); ++j) {
			if (dist(particles[i].x, particles[i].y, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f) <= sensor_range) {
				LandmarkObs obs;
				obs.x = map_landmarks.landmark_list[j].x_f;
				obs.y = map_landmarks.landmark_list[j].y_f;
				landmarks.push_back(obs);
			}
		}

		// Associate landmarks to map observations
		dataAssociation(landmarks, map_obs);

		// Multi-variate Gaussian
		double weight = 1;
		for (unsigned int j = 0; landmarks.size(); j++) {
			double min_d = 1e6;
			int min_k = -1;
			
			for (unsigned int k = 0; k < map_obs.size(); ++k) {
				// Find distance to nearest landmark
				if (map_obs[k].id == landmarks[j].id) {
					double s_dist = dist(landmarks[j].x, landmarks[j].y, map_obs[k].x, map_obs[k].y);
					if (s_dist < min_d) {
						min_d = s_dist;
						min_k = k;
					}
				}
			}

			if (min_k != -1) {
				// Initialize Multi-variate Gaussian 
				double x = map_obs[min_k].x;
				double y = map_obs[min_k].y;
				double mu_x = landmarks[j].x;
				double mu_y = landmarks[j].y;
				double sigma_x = std_landmark[0];
				double sigma_y = std_landmark[1];

				// Multi-variate Gaussian
				weight *= exp(-((x - mu_x) * (x - mu_x) / (2 * sigma_x * sigma_x) + (y - mu_y) * (y - mu_y) / (2 * sigma_y * sigma_y))) / (2 * M_PI * sigma_x * sigma_y);

			}
		}

		// Update paerticles
		weights.push_back(weight);
		particles[i].weight = weight;

	}
}

void ParticleFilter::resample() {
	/**********************************************************************************************
	* Resample particles with replacement with probability proportional to their weight. 
	* NOTE: You may find std::discrete_distribution helpful here.
	* http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	***********************************************************************************************/

	// Initialize variables
	auto first = weights.cbegin();
	auto last = weights.cend();
	auto count = distance(first, last);
	std::default_random_engine gen;

	// Initialize discrete distribution (See above)
	std::discrete_distribution<int> dist(count, -0.5, -0.5 + count, [&first](size_t i){
		return *std::next(first, i);
	});

	// Resample
	std::vector<Particle> resample;
	for (int i = 0; i < num_particles; ++i) {
		resample.push_back(particles[dist(gen)]);
	}
	
	particles = resample;

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
