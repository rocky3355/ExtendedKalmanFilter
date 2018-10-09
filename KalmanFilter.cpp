#include "KalmanFilter.h"

void KalmanFilter::Init(Eigen::VectorXd &x, Eigen::MatrixXd &P, Eigen::MatrixXd &F, 
                        Eigen::MatrixXd &H, Eigen::MatrixXd &R, Eigen::MatrixXd &Q) {
    this->x = x;
    this->P = P;
    this->F = F;
    this->H = H;
    this->R = R;
    this->Q = Q;
}

void KalmanFilter::Predict() {
    /**
     TODO:
    * predict the state
    */
}

void KalmanFilter::Update(const Eigen::VectorXd &z) {
    /**
     TODO:
    * update the state by using Kalman Filter equations
    */
}

void KalmanFilter::UpdateEKF(const Eigen::VectorXd &z) {
    /**
     TODO:
    * update the state by using Extended Kalman Filter equations
    */
}
