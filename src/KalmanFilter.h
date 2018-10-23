#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter {

public:
    // state vector
    Eigen::VectorXd x;
    // state covariance matrix
    Eigen::MatrixXd P;
    // state transition matrix
    Eigen::MatrixXd F;
    // process covariance matrix
    Eigen::MatrixXd Q;
    // measurement matrix
    Eigen::MatrixXd H;
    // measurement covariance matrix
    Eigen::MatrixXd R;

    void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in, Eigen::MatrixXd &Q_in);
    void Predict();
    void Update(const Eigen::VectorXd &z);
    void UpdateEKF(const Eigen::VectorXd &z);
    void UpdateXAndP(const Eigen::VectorXd &y);
};

#endif /* KALMAN_FILTER_H_ */
