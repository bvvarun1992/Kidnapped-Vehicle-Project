/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
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

static std::default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  // TODO: Set the number of particles

  //Creating normal distribution for x,y and theta
	std::normal_distribution<double> dist_x(x, std[0]);
	std::normal_distribution<double> dist_y(y, std[1]);
	std::normal_distribution<double> dist_theta(theta, std[2]);

  //Creating particles and initializing pos based on GPS info
  //Setting weight of all particles to 1
	for(int i = 0; i < num_particles; i++) {
		Particle p;
		p.id = i;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
		p.weight = 1.0;
		particles.push_back(p);
		weights.push_back(1.0);
	}
  //setting initialization flag true after initialization
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // TODO: Add measurements to each particle and add random Gaussian noise.
  // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
  //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  //  http://www.cplusplus.com/reference/random/default_random_engine/
  // define normal distributions for sensor noise

  //Creating normal distributions
  std::normal_distribution<double> dist_x(0, std_pos[0]);
  std::normal_distribution<double> dist_y(0, std_pos[1]);
  std::normal_distribution<double> dist_theta(0, std_pos[2]);

  for (int i = 0; i < num_particles; i++) {

    // Adding predicted x,y and theta at time t+1 to x,y and theta at time t
    if (std::fabs(yaw_rate) < 0.00001) {  
      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);
    } 
    else {
      particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
      particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
      particles[i].theta += yaw_rate * delta_t;
    }

    //Adding random noise to measurements
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

  for (unsigned int i=0;i<observations.size();i++){

    double distance_min = std::numeric_limits<double>::max();
    int id_min = -1;

    //Finding the closest predicted measurement
    for (unsigned int j = 0; j < predicted.size(); j++){
      
      //Computing distance between predicted measurement and observation measurement
      double distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);

      if (distance < distance_min) {
        distance_min = distance;
        id_min = predicted[j].id;
      }
    }

    //Assigning closest predicted measurement to observation measurement
    observations[i].id = id_min;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  for (int i = 0; i < num_particles; i++) {

    double p_x = particles[i].x;
    double p_y = particles[i].y;
    double p_theta = particles[i].theta;

    //Finding landmarks in range for the particle based on Lidar/Radar data
    vector<LandmarkObs> LandmarksInRange;
    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {

      float landmarkx = map_landmarks.landmark_list[j].x_f;
      float landmarky = map_landmarks.landmark_list[j].y_f;
      int landmarkid = map_landmarks.landmark_list[j].id_i;

      if (std::fabs(landmarkx - p_x) <= sensor_range && std::fabs(landmarky - p_y) <= sensor_range) {
        LandmarksInRange.push_back(LandmarkObs{ landmarkid, landmarkx, landmarky });
      }
    }

    //Transforming observation from car to map coordinates
    vector<LandmarkObs> observationsMap;
    for (unsigned int j = 0; j < observations.size(); j++) {

      double xmap = cos(p_theta)*observations[j].x - sin(p_theta)*observations[j].y + p_x;
      double ymap = sin(p_theta)*observations[j].x + cos(p_theta)*observations[j].y + p_y;

      observationsMap.push_back(LandmarkObs{ observations[j].id, xmap, ymap });
    }

    //Associating nearest landmark to observations
    dataAssociation(LandmarksInRange, observationsMap);

    //Calculating weight of the particle
    particles[i].weight = 1.0;

    for (unsigned int j = 0; j < observationsMap.size(); j++) {
      
      double obs_x, obs_y, pred_x, pred_y;
      obs_x = observationsMap[j].x;
      obs_y = observationsMap[j].y;

      int associated_prediction = observationsMap[j].id;

      for (unsigned int k = 0; k < LandmarksInRange.size(); k++) {
        if (LandmarksInRange[k].id == associated_prediction) {
          pred_x = LandmarksInRange[k].x;
          pred_y = LandmarksInRange[k].y;
        }
      }

      double gauss_norm = (1/(2*M_PI*std_landmark[0]*std_landmark[1]));
      double exponent = -(std::pow(pred_x-obs_x,2)/(2*std::pow(std_landmark[0], 2)) + (std::pow(pred_y-obs_y,2)/(2*std::pow(std_landmark[1], 2))));

      double weight = gauss_norm * std::exp(exponent);

      particles[i].weight *= weight;
    }
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  vector<double> weights;

  //Collecting weights of all the particles
  for (int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
  }
  
  //Calculating max weight of particles
  double MaxWeight = *std::max_element(weights.begin(), weights.end());
  
  std::uniform_int_distribution<int> dint(0, num_particles-1);
  std::uniform_real_distribution<double> ddouble(0.0, MaxWeight);
  
  //Resampling wheel
  int index = dint(gen);
  double beta = 0.0;

  vector<Particle> NewParticles;
  
  for (int i = 0; i < num_particles; i++) {
    
    beta += ddouble(gen) * 2.0;
    while (beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    NewParticles.push_back(particles[index]);
  }
  particles = NewParticles;
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