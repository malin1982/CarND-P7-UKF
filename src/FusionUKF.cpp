#include "FusionUKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionUKF::FusionUKF() {
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

  /**
  TODO:
    * Finish initializing the FusionUKF.
    * Set the process and measurement noises
  */

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  Hj_ << 0.5, 0.5, 0, 0,
         0.5, 0.5, 0, 0,
         0.1, 0.1, 0.7, 0.7;

}

/**
* Destructor.
*/
FusionUKF::~FusionUKF() {}

void FusionUKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ukf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    //cout << "UKF: " << endl;
    ukf_.x_ = VectorXd(4);
    ukf_.x_ << 0, 0, 0, 0;

    // Initial transition matrix, F
    ukf_.F_ = MatrixXd(4, 4);
    ukf_.F_ << 1, 0, 1, 0,
               0, 1, 0, 1,
               0, 0, 1, 0,
               0, 0, 0, 1;

    //state covariance matrix P
    ukf_.P_ = MatrixXd(4,4);
    ukf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;

    //measurement covariance matrix Q
    ukf_.Q_ = MatrixXd(4,4);
    ukf_.Q_ << 0, 0, 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0,
               0, 0, 0, 0;

    // process noise
    ukf_.v_ = VectorXd(4);
    ukf_.v_ << 0, 0, 0, 0;

    // measurement noise
    ukf_.w_ = VectorXd(4);
    ukf_.w_ << 0, 0, 0, 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rho_dot = measurement_pack.raw_measurements_[2];
      float px = rho * cos(phi);
      float py = rho * sin(phi);
      float vx = rho_dot * cos(phi);
      float vy = rho_dot * sin(phi);

      if (px == 0 or py == 0){
        return;
      }

      ukf_.x_ << px, py, vx, vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      if (measurement_pack.raw_measurements_[0] == 0 or measurement_pack.raw_measurements_[1] == 0) {
        return;
      }
      ukf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
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
   //compute the time elapsed between the current and previous measurements
 	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
 	previous_timestamp_ = measurement_pack.timestamp_;

  if (dt > 0.0001) {
   	float dt_2 = dt * dt;
   	float dt_3 = dt_2 * dt;
   	float dt_4 = dt_3 * dt;

    //Modify the F matrix so that the time is integrated
  	ukf_.F_(0, 2) = dt;
  	ukf_.F_(1, 3) = dt;

  	//set the process covariance matrix Q
    float noise_ax = 6.25;
    float noise_ay = 6.25;

  	ukf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
  			   0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
  			   dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
  			   0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

    ukf_.v_ << dt_2/2*noise_ax, dt_2/2*noise_ay, dt*noise_ax, dt*noise_ay;

    ukf_.Predict();
  }
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    // Radar updates
    Hj_ = tools.CalculateJacobian(ukf_.x_);
    ukf_.H_ = Hj_;
    ukf_.R_ = R_radar_;

    // Rho = Range
    double rho = sqrt(pow(ukf_.x_[0], 2) + pow(ukf_.x_[1], 2));

    // Phi = Bearing
    double phi = atan(ukf_.x_[1] / ukf_.x_[0]);

    // Rho_dot = Range rate
    double rho_dot = ((ukf_.x_[0] * ukf_.x_[2] + ukf_.x_[1] * ukf_.x_[3]) / (sqrt(pow(ukf_.x_[0], 2) + pow(ukf_.x_[1], 2))));

    MatrixXd zpred(3, 1);
    zpred << rho, phi, rho_dot;

    ukf_.UpdateUKF(measurement_pack.raw_measurements_, zpred);

  } else {
    // Laser updates
    ukf_.R_ = R_laser_;
    ukf_.H_ = H_laser_;
    ukf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  //cout << "x_ = " << ukf_.x_ << endl;
  //cout << "P_ = " << ukf_.P_ << endl;
}
