// Authors:
//  - Abhinav Goyal github.com/abnvgoyal
//  - Colin Faletto github.com/faletto

// General UKF info

// State Vector
//   x, y, z, (position)
//   u, v, w, (linear x/y/z velocity)
//   φ, θ, ψ, (roll, pitch, yaw)
//   p, q, r  (roll rate, pitch rate, yaw rate)

// Measurement Vector
//   lat, lon, alt, gs (latitude & longitude from GPS, altitude from baro, ground speed from GPS)
//   p, q, r (angular velocity from IMU)
//   ax, ay, az (linear acceleration from IMU)

#include <iostream>
#include <random>
#include <Eigen/Dense>
#include <cmath>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

double x_pos = 0.0;
double x_vel = 0.0;
std::random_device rd;

double x_pos_std_dev = 2.0;
double x_vel_std_dev = 0.5;
std::mt19937 gen(rd());
// Uniform distribution for velocity
std::uniform_real_distribution<double> distribution(0.0, 1.0);

// mass of drone
const double mass = 1.0;

const int dimension = 12;
const int measurement_dimension = 10;
// Change state vector to show actual initial state
Eigen::Matrix<double, dimension, 1> state_vector = Eigen::Matrix<double, dimension, 1>::Zero();
Eigen::Matrix<double, dimension, dimension> covariance = Eigen::Matrix<double, dimension, dimension>::Identity(dimension, dimension);

double generate_normal_random(double mean, double stddev) {
    std::normal_distribution<double> dist(mean, stddev);
    return dist(gen);
}

std::vector<double> generate_measurements() {
    double x_velocity = distribution(gen);
    x_vel = x_velocity;
    x_pos += x_velocity;
    double x_pos_measurement = generate_normal_random(x_pos, x_pos_std_dev);
    double x_velocity_measurement = generate_normal_random(x_velocity, x_vel_std_dev);
    return {x_pos_measurement, x_velocity_measurement};
}


//Generates sigma points for use in UKF off of covariance matrix and values for alpha, beta, and kappa. Generates 25 sigma points as we are using 12 dimensional state vector
std::vector<Eigen::Matrix<double, dimension, 1>> generate_sigma_points(Eigen::VectorXd mean, Eigen::Matrix<double, dimension, dimension> covariance, double alpha, double beta, double kappa) {
    int n = covariance.rows();
    double lambda = alpha * alpha * (n + kappa) - n;
    std::vector<Eigen::Matrix<double, dimension, 1>> sigma_points;
    sigma_points.push_back(mean);

    Eigen::Matrix<double, dimension, dimension> L = covariance.llt().matrixL(); // Lower triangular matrix
    Eigen::Matrix<double, dimension, dimension> displacement = L * std::sqrt(n + lambda);

    for (int i = 0; i < n; i++) {
        sigma_points.push_back(mean + displacement.col(i));
        sigma_points.push_back(mean - displacement.col(i));
    }
    return sigma_points;
}

//Goes along with sigma points
std::map<std::string, double> compute_weights(int n, double alpha, double beta, double kappa) {
    double lambda = alpha * alpha * (n + kappa) - n;
    std::map<std::string, double> weights;
    // mean weightage of mean sigma point
    weights["mean_mean_weight"] = lambda / (n + lambda);
    // covariance weightage of mean sigma point
    weights["mean_cov_weight"] = lambda / (n + lambda) + 1 - alpha * alpha + beta;
    // mean and covariance weightage of other sigma points
    weights["other_weights"] = 1 / (2 * (n + lambda));
    return weights;
}

//Predict what the next state will be based off current state and current inputs
Eigen::Matrix<double, dimension, 1> increment_state(Eigen::Matrix<double, dimension, 1> current, double dt, Eigen::Matrix3d J, double mass, Eigen::Vector3d moment_inputs, Eigen::Vector3d force_inputs) {
    Eigen::Vector3d angular_vel_old = current.tail<3>();
    Eigen::Vector3d angular_acc = J.inverse() * ((-angular_vel_old).cross(J * angular_vel_old) + moment_inputs);

    Eigen::Vector3d angular_pos_old = current.segment<3>(6);
    Eigen::Matrix3d angular_vel_transform;
    angular_vel_transform << 1, (sin(angular_pos_old.x())*tan(angular_pos_old.y())), (cos(angular_pos_old.x())*tan(angular_pos_old.y())),
                             0, (cos(angular_pos_old.x())), (-sin(angular_pos_old.x())),
                             0, (sin(angular_pos_old.x())/cos(angular_pos_old.y())), (cos(angular_pos_old.x())/cos(angular_pos_old.y()));
    Eigen::Vector3d angular_vel = angular_vel_transform * angular_vel_old;

    Eigen::Vector3d translation_vel_old = current.segment<3>(3);
    Eigen::Vector3d translation_acc_transform;
    translation_acc_transform << (angular_vel_old.z()*translation_vel_old.y() - angular_vel_old.y()*translation_vel_old.z()),
                                 (angular_vel_old.x()*translation_vel_old.z() - angular_vel_old.z()*translation_vel_old.x()),
                                 (angular_vel_old.y()*translation_vel_old.x() - angular_vel_old.x()*translation_vel_old.y());
    Eigen::Vector3d translation_acc = translation_acc_transform + (force_inputs / mass);

    Eigen::Matrix3d translational_vel_transform;
    translational_vel_transform << (cos(angular_pos_old.y())*cos(angular_pos_old.z())), 
                                   (sin(angular_pos_old.x())*sin(angular_pos_old.y())*cos(angular_pos_old.z()) - cos(angular_pos_old.x())*sin(angular_pos_old.z())),
                                   (cos(angular_pos_old.x())*sin(angular_pos_old.y())*cos(angular_pos_old.z()) + sin(angular_pos_old.x())*sin(angular_pos_old.z())),
                                   (cos(angular_pos_old.y())*sin(angular_pos_old.z())),
                                   (sin(angular_pos_old.x())*sin(angular_pos_old.y())*sin(angular_pos_old.z()) + cos(angular_pos_old.x())*cos(angular_pos_old.z())), 
                                   (cos(angular_pos_old.x())*sin(angular_pos_old.y())*sin(angular_pos_old.z()) - sin(angular_pos_old.x())*cos(angular_pos_old.z())),
                                   (-sin(angular_pos_old.y())), 
                                   (sin(angular_pos_old.x())*cos(angular_pos_old.y())), 
                                   (cos(angular_pos_old.x())*cos(angular_pos_old.y()));
    Eigen::Vector3d translation_vel = translational_vel_transform * translation_vel_old;

    Eigen::Matrix<double, dimension, 1> derivative(12);
    derivative.segment<3>(0) = translation_vel;
    derivative.segment<3>(3) = translation_acc; 
    derivative.segment<3>(6) = angular_vel;
    derivative.segment<3>(9) = angular_acc;
    Eigen::Matrix<double, dimension, 1> incremented = current + (derivative * dt);

    return incremented;
}

//Map the predicted state onto measurement space -- i.e, predict what the sensors will output
Eigen::Matrix<double, measurement_dimension, 1> measurement_function(Eigen::Matrix<double, dimension, 1> past, Eigen::Matrix<double, dimension, 1> current) {
    Eigen::Matrix<double, measurement_dimension, 1> measurement = Eigen::Matrix<double, measurement_dimension, 1>::Zero();

    // Extract state
    double x = current(0);
    double y = current(1);
    double z = current(2);
    double u = current(3);
    double v = current(4);
    double w = current(5);
    double phi   = current(6);
    double theta = current(7);
    double psi   = current(8);
    double p = current(9);
    double q = current(10);
    double r = current(11);

    // --- Rotation matrices ---
    Eigen::Matrix3d Rz;
    Rz << cos(psi), -sin(psi), 0,
          sin(psi),  cos(psi), 0,
          0,         0,        1;

    Eigen::Matrix3d Ry;
    Ry << cos(theta), 0, sin(theta),
          0,          1, 0,
         -sin(theta), 0, cos(theta);

    Eigen::Matrix3d Rx;
    Rx << 1, 0,          0,
          0, cos(phi),  -sin(phi),
          0, sin(phi),   cos(phi);

    // World-from-body rotation
    Eigen::Matrix3d R = Rz * Ry * Rx;

    // Body velocity to world velocity
    Eigen::Vector3d vel_body(u, v, w);
    Eigen::Vector3d vel_world = R * vel_body;

    // Predicted GPS groundspeed magnitude
    double gs = std::sqrt(vel_world(0)*vel_world(0) + vel_world(1)*vel_world(1));

    // Gravity vector in world frame
    Eigen::Vector3d g_world(0, 0, 9.81);
    Eigen::Vector3d g_body = R.transpose() * g_world;

    // Recompute body acceleration from state difference
    Eigen::Matrix<double, dimension, 1> diff = current - past;
    Eigen::Vector3d acc_body(
        diff(3),  // du/dt (approx)
        diff(4),  // dv/dt
        diff(5)   // dw/dt
    );

    // Specific force (accelerometer prediction)
    Eigen::Vector3d acc_meas = acc_body - g_body;

    // Convert x/y in meters to lat/lon using small-angle approx
    double R_earth = 6378137.0;
    double lat0 = 37.0 * M_PI / 180.0;
    double lon0 = -122.0 * M_PI / 180.0;

    double lat = lat0 + y / R_earth;
    double lon = lon0 + x / (R_earth * std::cos(lat0));

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

std::pair<Eigen::VectorXd, Eigen::MatrixXd> unscented_transform(std::vector<Eigen::VectorXd> sigma_points, std::map<std::string, double> weights, Eigen::MatrixXd process_noise) {
    Eigen::VectorXd mean = Eigen::VectorXd::Zero(sigma_points[0].size());
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(sigma_points[0].size(), sigma_points[0].size());
    for(int i = 0; i < sigma_points.size(); i++) {
        // Finding mean of new prior
        if(i == 0) {
            mean = weights["mean_mean_weight"] * sigma_points[i];
        } else {
            mean += weights["other_weights"] * sigma_points[i];
        }
    }

    // Finding covariance of new prior
    for(int i = 0; i < sigma_points.size(); i++) {
        if(i == 0) {
            Eigen::Matrix<double, dimension, 1> diff = sigma_points[i] - mean;
            covariance = weights["mean_cov_weight"] * diff * diff.transpose() + process_noise;
        } else {
            Eigen::Matrix<double, dimension, 1> diff = sigma_points[i] - mean;
            covariance += weights["other_weights"] * diff * diff.transpose() + process_noise;
        }
    }
    return {mean, covariance};
}

void unscented_kalman_filter(Eigen::Matrix<double, measurement_dimension, 1> measurements) {
    double alpha = 0.1;
    double beta = 2.0;
    double kappa = -1;
    std::vector<Eigen::Matrix<double, dimension, 1>> sigma_points = generate_sigma_points(state_vector, covariance, alpha, beta, kappa);
    std::map<std::string, double> weights = compute_weights(dimension, alpha, beta, kappa);

    Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
    
    Eigen::Vector3d moment_inputs = Eigen::Vector3d::Zero();
    Eigen::Vector3d force_inputs = Eigen::Vector3d::Zero();
    std::vector<Eigen::VectorXd> incremented;

    Eigen::Matrix<double, dimension, dimension> process_noise = Eigen::Matrix<double, dimension, dimension>::Zero();
    
    Eigen::Matrix<double, dimension, 1> prior_mean = Eigen::Matrix<double, dimension, 1>::Zero();
    Eigen::Matrix<double, dimension, dimension> prior_covariance = Eigen::Matrix<double, dimension, dimension>::Zero();
    
    // Projecting sigma points forward in time
    for(int i = 0; i < sigma_points.size(); i++) {
        incremented.push_back(increment_state(sigma_points[i], 0.1, J, mass, moment_inputs, force_inputs));
    }

    auto [prior_mean, prior_covariance] = unscented_transform(incremented, weights, process_noise);

    std::vector<Eigen::VectorXd> measurement_predictions;
    for(int i = 0; i < sigma_points.size(); i++) {
        measurement_predictions.push_back(measurement_function(sigma_points[i], incremented[i]));
    }
    auto [measurement_mean, measurement_covariance] = unscented_transform(measurement_predictions, weights, process_noise);
    auto residual = measurements - measurement_mean;
    auto cross_covariance = Eigen::MatrixXd::Zero(dimension, measurement_dimension);
    for(int i = 0; i < sigma_points.size(); i++) {
        Eigen::Matrix<double, dimension, 1> diff = incremented[i] - prior_mean;
        Eigen::Matrix<double, measurement_dimension, 1> diff2 = measurement_predictions[i] - measurement_mean;
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

Eigen::Matrix<double, dimension, 1>
estimate_state_with_real_measurements(
    double lat, double lon, double alt, double gs,
    double p, double q, double r,
    double ax, double ay, double az
) {
    Eigen::Matrix<double, measurement_dimension, 1> m;
    m(0) = lat;
    m(1) = lon;
    m(2) = alt;
    m(3) = gs;
    m(4) = p;
    m(5) = q;
    m(6) = r;
    m(7) = ax;
    m(8) = ay;
    m(9) = az;

    unscented_kalman_filter(m);
    return state_vector;
}


int main() {
    // int dimension = 2;
    // Eigen::VectorXd mean(dimension);
    // mean << 0, 0;

    // Eigen::MatrixXd covariance(dimension, dimension);
    // covariance << 2, 0.5,
    //               0.5, 3;

    // Eigen::MatrixXd J(3, 3);
    // J << 1, 0, 0,
    //      0, 1, 0,
    //      0, 0, 1;
    
    // std::vector<double> x, y;
    // for (auto& point : sigma) {
    //     x.push_back(point(0));
    //     y.push_back(point(1));
    // }

    // plt::scatter(x, y, 50);
    // plt::show();
    return 0;
}
