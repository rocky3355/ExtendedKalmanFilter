#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <vector>
#include <string>
#include <fstream>
#include "Eigen/Dense"
#include "Tools.h"
#include "KalmanFilter.h"
#include "MeasurementPackage.h"


class FusionEKF {

public:
    FusionEKF();
    void ProcessMeasurement(const MeasurementPackage &measurement);
    KalmanFilter Filter;

private:
    Tools _tools;
    bool _isInitialized;
    long long _previousTimestamp;
    Eigen::MatrixXd _RLaser;
    Eigen::MatrixXd _RRadar;
    Eigen::MatrixXd _HLaser;
    Eigen::MatrixXd _Hj;
};

#endif /* FusionEKF_H_ */