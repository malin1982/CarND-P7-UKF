#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* previous measurement
  MeasurementPackage previous_measurement_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* the current NIS for radar
  double NIS_radar_;

  ///* the current NIS for laser
  double NIS_laser_;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  void GenerateAugmentedSigmaPoints(const VectorXd &x, const MatrixXd &P, MatrixXd &Xsig_aug);
  void SigmaPointPrediction(double delta_t, const MatrixXd &Xsig_aug, MatrixXd &Xsig_pred);
    
  void PredictMeanAndCovariance(const MatrixXd &Xsig_pred, VectorXd &x, MatrixXd &P);
    
  void PredictLidarMeasurement(const MatrixXd &Xsig_pred, MatrixXd &Ysig, VectorXd &y_pred, MatrixXd &S);
  void PredictRadarMeasurement(const MatrixXd &Xsig_pred, MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S);
  void UpdateState(const VectorXd &z, const VectorXd &z_pred, const MatrixXd &S,
                     const MatrixXd &Xsig_pred, const MatrixXd &Zsig, VectorXd &x, MatrixXd &P);
};

// refer http://stackoverflow.com/questions/4633177/c-how-to-wrap-a-float-to-the-interval-pi-pi
// Bring the 'difference' between two angles into [-pi; pi].

template<typename T>
T normalizeRadianPi(T rad) {
    static const T PI2 = 2.*M_PI;
    // Copy the sign of the value in radians to the value of pi.
    T signed_pi = std::copysign(M_PI,rad);
    // Set the value of difference to the appropriate signed value between pi and -pi.
    rad = std::fmod(rad + signed_pi,(PI2)) - signed_pi;
    return rad;
}

#endif /* UKF_H */
