#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    // Set the number of particles
    num_particles = 20;
    std::normal_distribution<double> dist_x(0, std[0]);
    std::normal_distribution<double> dist_y(0, std[1]);
    std::normal_distribution<double> dist_theta(0, std[2]);
    std::default_random_engine gen;

    for (int i = 0; i < num_particles; ++i) {
      // Initialize particle position and noise
      Particle p;
      p.id = i;
      p.x = x + dist_x(gen);
      p.y = y + dist_y(gen);
      p.theta = theta + dist_theta(gen);
    
      // Initialize weight to 1
      p.weight = 1;
      weights.push_back(p.weight);
      particles.push_back(p);
    }

    is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // Random Gaqussian niose distribution
    std::normal_distribution<double> noise_x(0, std_pos[0]);
    std::normal_distribution<double> noise_y(0, std_pos[1]);
    std::normal_distribution<double> noise_theta(0, std_pos[2]);
    std::default_random_engine gen;

    for (unsigned int i=0; i < particles.size(); ++i) {
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
      // Add mniose
      particles[i].x = x_pred + noise_x(gen);
      particles[i].y = y_pred + noise_y(gen);
      particles[i].theta = theta_pred + noise_theta(gen);

    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    // Initialize measurement predictions
    for (unsigned i = 0; i < observations.size(); ++i) {
      double cur_dist = 1e6;
      int landmark = -1;

      for (unsigned j = 0; j < predicted.size(); ++j) {
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
    // Initialize weights
    weights.clear();
  
    for (unsigned i = 0; i < particles.size(); ++i) {
    // Convert observation onto map
      std::vector<LandmarkObs> map_obs;
      for (unsigned int j = 0; j < observations.size(); ++j) {
        if (dist(observations[j].x, observations[j].y, 0, 0) <= sensor_range) {
          LandmarkObs obs;
          obs.x = particles[i].x + observations[j].x * cos(particles[i].theta) - observations[j].y * sin(particles[i].theta);
          obs.y = particles[i].y + observations[j].x * sin(particles[i].theta) + observations[j].y * cos(particles[i].theta);
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
          obs.id = map_landmarks.landmark_list[j].id_i;
          landmarks.push_back(obs);
        }
      }

      // Associate landmarks to map observations
      dataAssociation(landmarks, map_obs);

      // Multi-variate Gaussian 
      double weight = 1;
      for (unsigned int j = 0; j < landmarks.size(); j++) {
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
          // Initialize Multi-variate Gussian
          double x = map_obs[min_k].x;
          double y = map_obs[min_k].y;
          double mu_x = landmarks[j].x;
          double mu_y = landmarks[j].y;
          double sigma_x = std_landmark[0];
          double sigma_y = std_landmark[1];
          
          weight *= exp(-((x - mu_x) * (x - mu_x) / (2 * sigma_x * sigma_x) + (y - mu_y) * (y - mu_y) / (2 * sigma_y * sigma_y))) / (2 * M_PI * sigma_x * sigma_y);
        }
      }

      // Update particles
      weights.push_back(weight);
      particles[i].weight = weight;

    }

}

void ParticleFilter::resample() {
    // Initialise variables
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
