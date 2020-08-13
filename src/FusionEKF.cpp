#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

const double  PI=3.14159265358979323846;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {

  cout << "initializing FusionEKF" << endl;
  cout << ekf_.x_ << endl;
  
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

/*
- Initialize the Kalman Filter
- Prepare the Q and F matrices for the prediction step
- Call the radar and lidar update functions.
*/


  /**
   * Finish initializing the FusionEKF.
   * Set the process and measurement noises
   */
  
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1,0,1,0,
    	     0,1,0,1,
    		 0,0,1,0,
    		 0,0,0,1;  
  ekf_.Q_ = MatrixXd(4,4);  
  ekf_.P_ = MatrixXd(4,4);
  ekf_.P_ << 0,0,0,0,
  			 0,0,0,0,
  			 0,0,1000.0,0,
  			 0,0,0,1000.0;
  H_laser_ << 1,0,0,0,
              0,1,0,0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {

    //cout << "initialize with first reading" << endl;

    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    
    // cout << ekf_.x_ << endl;
	//cout << measurement_pack.raw_measurements_ << endl;
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      //cout << "radar" << endl;

      double phi = measurement_pack.raw_measurements_[1];
      //cout << phi << endl;
      while (phi > PI) {
        phi = phi - (2 * PI);
      }
      while (phi < -PI) {
        phi = phi + (2 * PI);
      }
      //cout << phi << endl;      
      double rho = measurement_pack.raw_measurements_[0];
      ekf_.x_ << rho * cos(phi),
                 rho * sin(phi),
                 0,
                 0;  
      //cout << "radar EKF initialized" << endl;	  
      // jacobian (for radar) must be set with px, py, vx, vy
      Hj_ <<  tools.CalculateJacobian(ekf_.x_);
	  
      //cout << "radar step complete" << endl;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize state.
      //cout << "laser" << endl;

      ekf_.x_ << measurement_pack.raw_measurements_[0],
                 measurement_pack.raw_measurements_[1],
                 0,
                 0;
      //cout << "laser EKF initialized" << endl;	 
      
      //cout << "laser step complete" << endl;
    }

    previous_timestamp_ = measurement_pack.timestamp_;
    
    // done initializing, no need to predict or update
    is_initialized_ = true;

    
    return;
  }

  /**
   * Prediction
   */

  /**
   * Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  // compute time between current time and previous measurements
  // dt expressed in seconds
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1.0e6;
  previous_timestamp_ = measurement_pack.timestamp_;

  //cout << "measurement received" << endl;
  
  // modify F matrix with new time for prediction step
  // x' = F*x
  // P' = F*P*F_T + Q
  
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  //cout << "F updated" << endl;
  
  // for Q matrix
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  double noise_ax = 9.0;
  double noise_ay = 9.0;
  
  ekf_.Q_ << dt_4/4*noise_ax, 0.0, dt_3/2*noise_ax, 0.0,
  			 0.0, dt_4/4*noise_ay, 0.0, dt_3/2*noise_ay,
  			 dt_3/2*noise_ax, 0.0, dt_2*noise_ax, 0.0,
  			 0.0, dt_3/2*noise_ay, 0.0, dt_2*noise_ay;

  //cout << "Q updated" << endl;
  
  ekf_.Predict();
  //cout << "prediction" << endl;
  //cout << ekf_.x_ << endl;

  /**
   * Update
   */

  /**
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    //cout << "update radar" << endl;
	Hj_ <<  tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    //cout << "jacobian updated" << endl;
    ekf_.R_ = R_radar_;
    
    VectorXd z =  measurement_pack.raw_measurements_;
    double phi = measurement_pack.raw_measurements_[1];

    while (phi > PI) {
    	phi = phi - (2 * PI);
    }
    while (phi < -PI) {
    	phi = phi + (2 * PI);
    }
    z(1) = phi;    
 
    ekf_.UpdateEKF(z);
    //cout << "update complete" << endl;  
    
  } else {
    // Laser updates
    //cout << "update laser" << endl;
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    
    ekf_.Update(measurement_pack.raw_measurements_);    
    //cout << "update complete" << endl;      
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
