// ukf.h
#ifndef UKF_H
#define UKF_H

#include <vector>
#include <map>
#include <Eigen/Dense>

// Function declarations
double generate_normal_random(double mean, double stddev);

std::vector<double> generate_measurements();

std::vector<Eigen::Matrix<double, 12, 1>> generate_sigma_points(
    Eigen::VectorXd mean,
    Eigen::Matrix<double, 12, 12> covariance,
    double alpha,
    double beta,
    double kappa
);

std::map<std::string, double> compute_weights(int n, double alpha, double beta, double kappa);

Eigen::Matrix<double, 12, 1> increment_state(
    Eigen::Matrix<double, 12, 1> current,
    double dt,
    Eigen::Matrix3d J,
    double mass,
    Eigen::Vector3d moment_inputs,
    Eigen::Vector3d force_inputs
);

Eigen::Matrix<double, 10, 1> measurement_function(
    Eigen::Matrix<double, 12, 1> past,
    Eigen::Matrix<double, 12, 1> current
);

std::pair<Eigen::VectorXd, Eigen::MatrixXd> unscented_transform(
    std::vector<Eigen::VectorXd> sigma_points,
    std::map<std::string, double> weights,
    Eigen::MatrixXd process_noise
);

void unscented_kalman_filter(Eigen::Matrix<double, 10, 1> measurements);

#endif // UKF_H