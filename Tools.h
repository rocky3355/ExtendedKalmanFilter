#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

class Tools {

public:
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &groundTruth);
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& xState);
};

#endif /* TOOLS_H_ */
