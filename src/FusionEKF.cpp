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

    // Covariance matrices
    _RLaser << 0.0225, 0,
        0, 0.0225;

    _RRadar << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

    _HLaser << 1, 0, 0, 0,
               0, 1, 0, 0;

    _noiseAx = 9.0;
    _noiseAy = 9.0;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement) {
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!_isInitialized) {
        Eigen::VectorXd x = Eigen::VectorXd(4);

        if (measurement.SensorType == MeasurementPackage::RADAR) {
            double rho = measurement.RawMeasurements(0);
  	        double phi = measurement.RawMeasurements(1);
  	        double rho_dot = measurement.RawMeasurements(2);

            double pX = rho * cos(phi);
            double pY = rho * sin(phi);
            double vX = rho_dot * cos(phi);
            double vY = rho_dot * sin(phi);
            x << pX, pY, vX, vY;
        }
        else if (measurement.SensorType == MeasurementPackage::LASER) {
            double pX = measurement.RawMeasurements(0);
            double pY = measurement.RawMeasurements(1);
            x << pX, pY, 0, 0;
        }

        Eigen::MatrixXd P(4, 4);
        P << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

        Eigen::MatrixXd F(4, 4);
        F << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;

        Eigen::MatrixXd Q(4,4);
        Filter.Init(x, P, F, Q);
        _previousTimestamp = measurement.Timestamp;
        _isInitialized = true;
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/
    // dt is in seconds
    float dt = (measurement.Timestamp - _previousTimestamp) / 1000000.0;
	_previousTimestamp = measurement.Timestamp;

    float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;

	// Modify the F matrix so that the time is integrated
	Filter.F(0, 2) = dt;
	Filter.F(1, 3) = dt;

	// Set the process covariance matrix Q
	Filter.Q = Eigen::MatrixXd(4, 4);
	Filter.Q << dt_4 / 4 * _noiseAx, 0, dt_3 / 2 * _noiseAx, 0,
			    0, dt_4 / 4 * _noiseAy, 0, dt_3 / 2 * _noiseAy,
			    dt_3 / 2 * _noiseAx, 0, dt_2 * _noiseAx, 0,
			    0, dt_3 / 2 * _noiseAy, 0, dt_2 * _noiseAy;

    Filter.Predict();

    /*****************************************************************************
     *  Update
     ****************************************************************************/
    if (measurement.SensorType == MeasurementPackage::RADAR) {
        // Radar updates
        Filter.H = Tools::CalculateJacobian(Filter.x);
  	    Filter.R = _RRadar;
        Filter.UpdateEKF(measurement.RawMeasurements);
    } else {
        // Laser updates
        Filter.H = _HLaser;
  	    Filter.R = _RLaser;
        Filter.Update(measurement.RawMeasurements);
    }
}