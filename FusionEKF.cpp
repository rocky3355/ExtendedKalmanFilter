#include "FusionEKF.h"
#include "Tools.h"
#include "Eigen/Dense"
#include <iostream>

FusionEKF::FusionEKF() {
    _isInitialized = false;
    _previousTimestamp = 0;

    _RLaser = Eigen::MatrixXd(2, 2);
    _RRadar = Eigen::MatrixXd(3, 3);
    _HLaser = Eigen::MatrixXd(2, 4);
    _Hj = Eigen::MatrixXd(3, 4);

    _RLaser << 0.0225, 0,
        0, 0.0225;

    _RRadar << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

    /**
     TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
    */
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement) {
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/

    if (!_isInitialized) {
        /**
        TODO:
            * Initialize the state ekf_.x_ with the first measurement.
            * Create the covariance matrix.
            * Remember: you'll need to convert radar from polar to cartesian coordinates.
        */
        std::cout << "EKF: " << std::endl;
        Filter.x = Eigen::VectorXd(4);
        Filter.x << 1, 1, 1, 1;

        if (measurement.SensorType == MeasurementPackage::RADAR) {
            /**
             Convert radar from polar to cartesian coordinates and initialize state.
            */
        }
        else if (measurement.SensorType == MeasurementPackage::LASER) {
            /**
             Initialize state.
            */
        }

        _isInitialized = true;
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    /**
     TODO:
        * Update the state transition matrix F according to the new elapsed time.
        - Time is measured in seconds.
        * Update the process noise covariance matrix.
        * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
    */

    Filter.Predict();

    /*****************************************************************************
     *  Update
     ****************************************************************************/

    /**
     TODO:
        * Use the sensor type to perform the update step.
        * Update the state and covariance matrices.
    */

    if (measurement.SensorType == MeasurementPackage::RADAR) {
        // Radar updates
    } else {
        // Laser updates
    }

    // print the output
    std::cout << "x = " << Filter.x << std::endl;
    std::cout << "P = " << Filter.P << std::endl;
}