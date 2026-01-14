// Authors:
//  - Abhinav Goyal github.com/abnvgoyal
//  - Colin Faletto github.com/faletto

// Jupyter Notebook Link:
// https://nbviewer.org/github/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/10-Unscented-Kalman-Filter.ipynb

#include <iostream>
#include <random>
#include <ArduinoEigenDense.h>
#include <cmath>
#include <vector>
#include <map>
#include "structs.h"

float x_pos = 0.0;
float x_vel = 0.0;
std::random_device rd;

float x_pos_std_dev = 2.0;
float x_vel_std_dev = 0.5;

// Initializes random number generator
std::mt19937 gen(rd());

// Uniform distribution for velocity
std::uniform_real_distribution<float> distribution(0.0, 1.0);

// NOTE: PLACEHOLDER VALUE
const float mass = 1.0;

float G_EARTH = 9.81;
float R_EARTH = 6378137.0;

// State Vector contains 12 values
//   x, y, z, (position)
//   u, v, w, (linear x/y/z velocity)
//   φ, θ, ψ, (roll, pitch, yaw)
//   p, q, r  (roll rate, pitch rate, yaw rate)
const int dimension = 12;

// Measurement Vector contains 10 values
//   lat, lon, alt, gs (latitude & longitude from GPS, altitude from baro, ground speed from GPS)
//   p, q, r (angular velocity from IMU)
//   ax, ay, az (linear acceleration from IMU)
const int measurement_dimension = 10;


Eigen::Matrix<float, dimension, 1> state_vector = Eigen::Matrix<float, dimension, 1>::Zero();
Eigen::Matrix<float, dimension, dimension> covariance = Eigen::Matrix<float, dimension, dimension>::Identity(dimension, dimension);

// Generates a random number from a normal / Gaussian distribution with
// the specified mean value and standard deviation.
// https://en.wikipedia.org/wiki/Normal_distribution
float generate_normal_random(float mean, float stddev) {
    std::normal_distribution<float> dist(mean, stddev);
    return dist(gen);
}


// Generates x velocity and position values with a simulated error from a normal distribution.
// This method is not currently referenced anywhere.
// std::vector<float> generate_measurements() {
//     float x_velocity = distribution(gen);
//     x_vel = x_velocity;
//     x_pos += x_velocity;
//     float x_pos_measurement = generate_normal_random(x_pos, x_pos_std_dev);
//     float x_velocity_measurement = generate_normal_random(x_velocity, x_vel_std_dev);
//     return {x_pos_measurement, x_velocity_measurement};
// }


// Generates sigma points for use in UKF off of covariance matrix and values for alpha, beta, and kappa. 
// Generates 25 sigma points as we are using a 12-dimensional state vector.
// Jupyter Notebook - Implementation of the UKF - Sigma Points
std::vector<Eigen::Matrix<float, dimension, 1>> generate_sigma_points(Eigen::VectorXf mean, 
    Eigen::Matrix<float, dimension, dimension> covariance, float alpha, float beta, 
    float kappa) {

    int n = covariance.rows();
    float lambda = alpha * alpha * (n + kappa) - n;
    std::vector<Eigen::Matrix<float, dimension, 1>> sigma_points;
    sigma_points.push_back(mean);

    Eigen::Matrix<float, dimension, dimension> L = 
    covariance.llt().matrixL(); // Lower triangular matrix
    Eigen::Matrix<float, dimension, dimension> displacement = L * std::sqrt(n + lambda);

    for (int i = 0; i < n; i++) {
        sigma_points.push_back(mean + displacement.col(i));
        sigma_points.push_back(mean - displacement.col(i));
    }
    return sigma_points;
}

// Computes weights for the sigma points.
// Jupyter Notebook - Implementation of the UKF - Weights
std::map<std::string, float> compute_weights(int n, float alpha, float beta, float kappa) {
    float lambda = alpha * alpha * (n + kappa) - n;
    std::map<std::string, float> weights;
    // mean weightage of mean sigma point
    weights["mean_mean_weight"] = lambda / (n + lambda);
    // covariance weightage of mean sigma point
    weights["mean_cov_weight"] = lambda / (n + lambda) + 1 - alpha * alpha + beta;
    // mean and covariance weightage of other sigma points
    weights["other_weights"] = 1 / (2 * (n + lambda));
    return weights;
}

// Returns a vector with the rates of change of every value in the state vector according to the kinematic model.
// https://drive.google.com/file/d/1BOwGqoJ2WjiIUYDA8p77TGwV-Ttrd4hc/view - Slide 15
// Parameters:
//  current - The current state vector
//  J - Inertia matrix, can be calculated with the aircraft CAD
//  mass - mass of aircraft
//  moment_inputs - externally applied moments to aircraft
//  force_inputs - externally applied forces to aircraft
Eigen::Matrix<float, dimension, 1> derivative_function(Eigen::Matrix<float, dimension, 1> current, Eigen::Matrix3f J, float mass, Eigen::Vector3f moment_inputs, Eigen::Vector3f force_inputs) {

    Eigen::Vector3f angular_vel_old = current.tail<3>();
    Eigen::Vector3f angular_acc = J.inverse() * ((-angular_vel_old).cross(J * angular_vel_old) + moment_inputs);

    Eigen::Vector3f angular_pos_old = current.segment<3>(6);
    Eigen::Matrix3f angular_vel_transform;
    angular_vel_transform << 1, (sin(angular_pos_old.x())*tan(angular_pos_old.y())), (cos(angular_pos_old.x())*tan(angular_pos_old.y())),
                             0, (cos(angular_pos_old.x())), (-sin(angular_pos_old.x())),
                             0, (sin(angular_pos_old.x())/cos(angular_pos_old.y())), (cos(angular_pos_old.x())/cos(angular_pos_old.y()));
    Eigen::Vector3f angular_vel = angular_vel_transform * angular_vel_old;

    Eigen::Vector3f translation_vel_old = current.segment<3>(3);
    Eigen::Vector3f translation_acc_transform;
    translation_acc_transform << (angular_vel_old.z()*translation_vel_old.y() - angular_vel_old.y()*translation_vel_old.z()),
                                 (angular_vel_old.x()*translation_vel_old.z() - angular_vel_old.z()*translation_vel_old.x()),
                                 (angular_vel_old.y()*translation_vel_old.x() - angular_vel_old.x()*translation_vel_old.y());
    Eigen::Vector3f translation_acc = translation_acc_transform + (force_inputs / mass);

    Eigen::Matrix3f translational_vel_transform;
    translational_vel_transform << (cos(angular_pos_old.y())*cos(angular_pos_old.z())), 
                                   (sin(angular_pos_old.x())*sin(angular_pos_old.y())*cos(angular_pos_old.z()) - cos(angular_pos_old.x())*sin(angular_pos_old.z())),
                                   (cos(angular_pos_old.x())*sin(angular_pos_old.y())*cos(angular_pos_old.z()) + sin(angular_pos_old.x())*sin(angular_pos_old.z())),
                                   (cos(angular_pos_old.y())*sin(angular_pos_old.z())),
                                   (sin(angular_pos_old.x())*sin(angular_pos_old.y())*sin(angular_pos_old.z()) + cos(angular_pos_old.x())*cos(angular_pos_old.z())), 
                                   (cos(angular_pos_old.x())*sin(angular_pos_old.y())*sin(angular_pos_old.z()) - sin(angular_pos_old.x())*cos(angular_pos_old.z())),
                                   (-sin(angular_pos_old.y())), 
                                   (sin(angular_pos_old.x())*cos(angular_pos_old.y())), 
                                   (cos(angular_pos_old.x())*cos(angular_pos_old.y()));
    Eigen::Vector3f translation_vel = translational_vel_transform * translation_vel_old;

    Eigen::Matrix<float, dimension, 1> derivative(12);
    derivative.segment<3>(0) = translation_vel;
    derivative.segment<3>(3) = translation_acc; 
    derivative.segment<3>(6) = angular_vel;
    derivative.segment<3>(9) = angular_acc;

    return derivative;
}



// Predict what the next state will be using the RK4 algorithm.
// https://lpsa.swarthmore.edu/NumInt/NumIntFourth.html
// Parameters:
//  current - The current state vector
//  h - The step size for RK4
//  J - Inertia matrix, can be calculated with the aircraft CAD
//  mass - mass of aircraft
//  moment_inputs - externally applied moments to aircraft
//  force_inputs - externally applied forces to aircraft
Eigen::Matrix<float, dimension, 1> increment_state(Eigen::Matrix<float, dimension, 1> current, float h, Eigen::Matrix3f J, float mass, Eigen::Vector3f moment_inputs, Eigen::Vector3f force_inputs) {

    Eigen::Matrix<float, dimension, 1> k1 = derivative_function(current, J, mass, moment_inputs, force_inputs);
    Eigen::Matrix<float, dimension, 1> k2 = derivative_function(current + k1 * h/2, J, mass, moment_inputs, force_inputs);
    Eigen::Matrix<float, dimension, 1> k3 = derivative_function(current + k2 * h/2, J, mass, moment_inputs, force_inputs);
    Eigen::Matrix<float, dimension, 1> k4 = derivative_function(current + k3 * h, J, mass, moment_inputs, force_inputs);
    
    Eigen::Matrix<float, dimension, 1> incremented = current + ( h / (6.0) * (k1 + 2 * k2 + 2 * k3 + k4));

    return incremented;
}


// Maps the predicted state onto measurement space -- i.e. predict what the sensors will output
Eigen::Matrix<float, measurement_dimension, 1> measurement_function(Eigen::Matrix<float, dimension, 1> past, Eigen::Matrix<float, dimension, 1> current) {
    Eigen::Matrix<float, measurement_dimension, 1> measurement = Eigen::Matrix<float, measurement_dimension, 1>::Zero();

    // Extract state from current state vector
    float x = current(0);
    float y = current(1);
    float z = current(2);
    float u = current(3);
    float v = current(4);
    float w = current(5);
    float phi   = current(6);
    float theta = current(7);
    float psi   = current(8);
    float p = current(9);
    float q = current(10);
    float r = current(11);

    // --- Converts pitch / roll / yaw into rotation matrices ---
    // https://msl.cs.uiuc.edu/planning/node102.html

    // Yaw
    Eigen::Matrix3f Rz;
    Rz << cos(psi), -sin(psi), 0,
          sin(psi),  cos(psi), 0,
          0,         0,        1;

    // Pitch
    Eigen::Matrix3f Ry;
    Ry << cos(theta), 0, sin(theta),
          0,          1, 0,
         -sin(theta), 0, cos(theta);

    // Roll
    Eigen::Matrix3f Rx;
    Rx << 1, 0,          0,
          0, cos(phi),  -sin(phi),
          0, sin(phi),   cos(phi);

    // World-from-body rotation
    // Rotation follows yaw-pitch-roll (Tait-Bryan angle) order
    //https://en.wikipedia.org/wiki/Euler_angles#Chained_rotations_equivalence
    Eigen::Matrix3f R = Rz * Ry * Rx;

    // Converts body velocity to world velocity
    Eigen::Vector3f vel_body(u, v, w);
    Eigen::Vector3f vel_world = R * vel_body;

    // Predicts GPS groundspeed magnitude using Pythagorean theorem
    float gs = std::sqrt(vel_world(0) * vel_world(0) + vel_world(1) * vel_world(1));

    // Gravity vector in world frame
    Eigen::Vector3f g_world(0, 0, G_EARTH);
    Eigen::Vector3f g_body = R.transpose() * g_world;

    // Recompute body acceleration from state difference
    Eigen::Matrix<float, dimension, 1> diff = current - past;
    Eigen::Vector3f acc_body(
        diff(3),  // du/dt (approx)
        diff(4),  // dv/dt
        diff(5)   // dw/dt
    );

    // Specific force (accelerometer prediction)
    Eigen::Vector3f acc_meas = acc_body - g_body;

    // Convert x/y in meters to lat/lon using small-angle approx
    float lat0 = 37.0 * M_PI / 180.0;
    float lon0 = -122.0 * M_PI / 180.0;
    float lat = lat0 + y / R_EARTH;
    float lon = lon0 + x / (R_EARTH * std::cos(lat0));

    // Fill measurement vector
    measurement(0) = lat;     // latitude
    measurement(1) = lon;     // longitude
    measurement(2) = z;       // barometer altitude
    measurement(3) = gs;      // groundspeed
    measurement(4) = p;       // gyro x
    measurement(5) = q;       // gyro y
    measurement(6) = r;       // gyro z
    measurement(7) = acc_meas(0); // accel x
    measurement(8) = acc_meas(1); // accel y
    measurement(9) = acc_meas(2); // accel z

    return measurement;
}


// Finds the mean and covariance for the sigma points.
// Jupyter Notebook - The Unscented Transform
std::pair<Eigen::VectorXf, Eigen::MatrixXf> unscented_transform(std::vector<Eigen::VectorXf> sigma_points, std::map<std::string, float> weights, Eigen::MatrixXf process_noise) {
    Eigen::VectorXf mean = Eigen::VectorXf::Zero(sigma_points[0].size());
    Eigen::MatrixXf covariance = Eigen::MatrixXf::Zero(sigma_points[0].size(), sigma_points[0].size());
    for(size_t i = 0; i < sigma_points.size(); i++) {
        // Finding mean of new prior
        if(i == 0) {
            mean = weights["mean_mean_weight"] * sigma_points[i];
        } else {
            mean += weights["other_weights"] * sigma_points[i];
        }
    }

    // Finding covariance of new prior
    for(size_t i = 0; i < sigma_points.size(); i++) {
        if(i == 0) {
            Eigen::Matrix<float, dimension, 1> diff = sigma_points[i] - mean;
            covariance = weights["mean_cov_weight"] * diff * diff.transpose() + process_noise;
        } else {
            Eigen::Matrix<float, dimension, 1> diff = sigma_points[i] - mean;
            covariance += weights["other_weights"] * diff * diff.transpose() + process_noise;
        }
    }
    return {mean, covariance};
}

// Implements the UKF to predict the next state vector.
// Jupyter Notebook - Impementation of the UKF
void unscented_kalman_filter(Eigen::Matrix<float, measurement_dimension, 1> measurements) {
    float alpha = 0.1;
    float beta = 2.0;
    float kappa = -1;
    std::vector<Eigen::Matrix<float, dimension, 1>> sigma_points = generate_sigma_points(state_vector, covariance, alpha, beta, kappa);
    std::map<std::string, float> weights = compute_weights(dimension, alpha, beta, kappa);

    // NOTE: PLACEHOLDER VALUES
    Eigen::Matrix3f J = Eigen::Matrix3f::Identity();
    Eigen::Vector3f moment_inputs = Eigen::Vector3f::Zero();
    Eigen::Vector3f force_inputs = Eigen::Vector3f::Zero();


    
    // NOTE: PLACEHOLDER VALUE
    Eigen::Matrix<float, dimension, dimension> process_noise = Eigen::Matrix<float, dimension, dimension>::Zero();
    
    
    Eigen::Matrix<float, dimension, 1> prior_mean = Eigen::Matrix<float, dimension, 1>::Zero();
    Eigen::Matrix<float, dimension, dimension> prior_covariance = Eigen::Matrix<float, dimension, dimension>::Zero();
    
    // Projects sigma points forward in time
    std::vector<Eigen::VectorXf> incremented;
    for(size_t i = 0; i < sigma_points.size(); i++) {
        incremented.push_back(increment_state(sigma_points[i], 0.1, J, mass, moment_inputs, force_inputs));
    }
    std::tie(prior_mean, prior_covariance) = unscented_transform(incremented, weights, process_noise);

    // Projects predicted measurements forward in time
    std::vector<Eigen::VectorXf> measurement_predictions;
    for(size_t i = 0; i < sigma_points.size(); i++) {
        measurement_predictions.push_back(measurement_function(sigma_points[i], incremented[i]));
    }
    auto [measurement_mean, measurement_covariance] = unscented_transform(measurement_predictions, weights, process_noise);

    auto residual = measurements - measurement_mean;

    // Calculates cross covariance of sigma points
    Eigen::Matrix<float, dimension, measurement_dimension> cross_covariance = Eigen::MatrixXf::Zero(dimension, measurement_dimension);
    for(size_t i = 0; i < sigma_points.size(); i++) {
        Eigen::Matrix<float, dimension, 1> diff = incremented[i] - prior_mean;
        Eigen::Matrix<float, measurement_dimension, 1> diff2 = measurement_predictions[i] - measurement_mean;
        if(i == 0) {
            cross_covariance += weights["mean_cov_weight"] * diff * diff2.transpose();
        } else {
            cross_covariance += weights["other_weights"] * diff * diff2.transpose();
        }
    }

    auto kalman_gain = cross_covariance * measurement_covariance.inverse();
    state_vector =  prior_mean + kalman_gain * residual;
    covariance = prior_covariance - kalman_gain * measurement_covariance * kalman_gain.transpose();
}

StateVector_t estimate_state_with_real_measurements(
    float lat, float lon, float alt, float gs,
    float p, float q, float r,
    float ax, float ay, float az
) {
    Eigen::Matrix<float, measurement_dimension, 1> measurements;
    measurements << lat, lon, alt, gs, p, q, r, ax, ay, az;
    unscented_kalman_filter(measurements);
     return {
        state_vector(0), state_vector(1), state_vector(2),
        state_vector(3), state_vector(4), state_vector(5),
        state_vector(6), state_vector(7), state_vector(8),
        state_vector(9), state_vector(10), state_vector(11)
    };


}
