#include <iostream>
#include "Tools.h"

Eigen::VectorXd Tools::CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &groundTruth) {
    Eigen::VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	if(estimations.size() != groundTruth.size() || estimations.size() == 0){
		std::cout << "Invalid estimation or ground_truth data" << std::endl;
		return rmse;
	}

	for(uint i = 0; i < estimations.size(); ++i){
		Eigen::VectorXd residual = estimations[i] - groundTruth[i];
		residual = residual.array() * residual.array();
		rmse += residual;
	}

	rmse = rmse / estimations.size();
	rmse = rmse.array().sqrt();

	return rmse;
}

Eigen::MatrixXd Tools::CalculateJacobian(const Eigen::VectorXd& xState) {
    Eigen::MatrixXd Hj(3,4);

    float px = xState(0);
	float py = xState(1);
	float vx = xState(2);
	float vy = xState(3);

	float c1 = px * px + py * py;
	float c2 = sqrt(c1);
	float c3 = (c1 * c2);

	if(fabs(c1) < 0.0001){
		std::cout << "CalculateJacobian() - Error - Division by Zero" << std::endl;
		return Hj;
	}

	Hj << (px / c2), (py / c2), 0, 0,
		  -(py / c1), (px / c1), 0, 0,
		  py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

	return Hj;
}
