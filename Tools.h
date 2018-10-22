#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

class Tools {

public:
    static Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &groundTruth);
    static Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& xState);
};

#endif /* TOOLS_H_ */
