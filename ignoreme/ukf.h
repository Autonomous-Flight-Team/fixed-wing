// ukf.h
#ifndef UKF_H
#define UKF_H

#include <vector>
#include <map>
#include <ArduinoEigenDense.h>
#include "structs.h"
// Function declarations
float generate_normal_random(float mean, float stddev);

std::vector<float> generate_measurements();

std::vector<Eigen::Matrix<float, 12, 1>> generate_sigma_points(
    Eigen::VectorXd mean,
    Eigen::Matrix<float, 12, 12> covariance,
    float alpha,
    float beta,
    float kappa
);

std::map<std::string, float> compute_weights(int n, float alpha, float beta, float kappa);

Eigen::Matrix<float, 12, 1> increment_state(
    Eigen::Matrix<float, 12, 1> current,
    float dt,
    Eigen::Matrix3d J,
    float mass,
    Eigen::Vector3d moment_inputs,
    Eigen::Vector3d force_inputs
);

Eigen::Matrix<float, 10, 1> measurement_function(
    Eigen::Matrix<float, 12, 1> past,
    Eigen::Matrix<float, 12, 1> current
);

std::pair<Eigen::VectorXd, Eigen::MatrixXd> unscented_transform(
    std::vector<Eigen::VectorXd> sigma_points,
    std::map<std::string, float> weights,
    Eigen::MatrixXd process_noise
);

void unscented_kalman_filter(Eigen::Matrix<float, 10, 1> measurements);

StateVector_t estimate_state_with_real_measurements(
    float lat, float lon, float alt, float gs,
    float p, float q, float r,
    float ax, float ay, float az
);

#endif // UKF_H