#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage {

public:
    long long Timestamp;

    enum SensorType{
        LASER,
        RADAR
    } SensorType;

    Eigen::VectorXd RawMeasurements;
};

#endif /* MEASUREMENT_PACKAGE_H_ */
