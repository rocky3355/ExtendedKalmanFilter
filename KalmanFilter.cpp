#include "KalmanFilter.h"

void KalmanFilter::Init(Eigen::VectorXd &x, Eigen::MatrixXd &P, Eigen::MatrixXd &F, Eigen::MatrixXd &Q) {
    this->x = x;
    this->P = P;
    this->F = F;
    this->Q = Q;
}

void KalmanFilter::Predict() {
    x = F * x;
	Eigen::MatrixXd Ft = F.transpose();
	P = F * P * Ft + Q;
}

void KalmanFilter::Update(const Eigen::VectorXd &z) {
    Eigen::VectorXd y = z - H * x;
    UpdateWithY(y);
}

void KalmanFilter::UpdateEKF(const Eigen::VectorXd &z) {
    /**
     TODO:
    * update the state by using Extended Kalman Filter equations
    */
    double px = x(0);
    double py = x(1);
    double vx = x(2);
    double vy = x(3);

    double rho = sqrt(px*px + py*py);
    double theta = atan2(py, px);
    double rho_dot = (px*vx + py*vy) / rho;
    Eigen::VectorXd h = Eigen::VectorXd(3);
    h << rho, theta, rho_dot;
    Eigen::VectorXd y = z - h;
    while (y(1) > M_PI || y(1) < -M_PI) {
        if ( y(1) > M_PI ) {
            y(1) -= M_PI;
        } else {
            y(1) += M_PI;
        }
    }
    UpdateWithY(y);
}

// TODO: RENAME!!!!!!!
void KalmanFilter::UpdateWithY(const Eigen::VectorXd &y) {
  Eigen::MatrixXd Ht = H.transpose();
  Eigen::MatrixXd S = H * P * Ht + R;
  Eigen::MatrixXd Si = S.inverse();
  Eigen::MatrixXd K =  P * Ht * Si;
  // New state
  x = x + (K * y);
  int xSize = x.size();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(xSize, xSize);
  P = (I - K * H) * P;
}
