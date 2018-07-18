#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ << 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  //create sigma point matrix
  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create matrix with predicted sigma points as columns
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  //define spreading parameter
  lambda_ = 3 - n_aug_;

  //create vector for weights_
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {  //2n+1 weights_
    weights_(i) = 0.5 / (n_aug_ + lambda_);
  }

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.5;  //1.5 // 0.3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.65;  //0.5;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  // the current NIS for radar
  NIS_radar_ = 0.0;

  // the current NIS for laser
  NIS_laser_ = 0.0;

  // previous timestamp
  previous_timestamp_ = 0;

  // time when the state is true, in us
  time_us_ = 0;

  // enable debug information
  debug = true;
}

UKF::~UKF() {
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

  if (!is_initialized_) {
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      double rho = meas_package.raw_measurements_[0];  // range
      double phi = meas_package.raw_measurements_[1];  // bearing
      double rho_dot = meas_package.raw_measurements_[2];  // velocity of rh
      double x = rho * cos(phi);
      double y = rho * sin(phi);
      double vx = rho_dot * cos(phi);
      double vy = rho_dot * sin(phi);
      double v = sqrt(vx * vx + vy * vy);
      x_ << x, y, v, 0, 0;
    } else {
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
    }

    // Initializing, no need to predict or update
    previous_timestamp_ = meas_package.timestamp_;
    is_initialized_ = true;

    if (debug) {
      cout << "UKF has been initialized!" << endl;
    }

    return;
  }

  // Compute the time elapsed between the current and previous measurements
  double dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;  //dt - expressed in seconds
  previous_timestamp_ = meas_package.timestamp_;

  if (debug) {
    cout << " dt = " << dt << endl;
    cout << " Starting state = " << x_ << endl;
    cout << " Starting covariance = " << P_ << endl;
  }

  // Generate Sigma Points
  GenerateAugmentedSigmaPoints();
  //cout << "  Sigma Points = " << Xsig_aug_ << endl;

  // Predict Sigma Points
  SigmaPointPrediction(dt);
  //cout << "  Predicted Sigma Points = " << Xsig_pred_ << endl;

  // Predict Mean and Covariance
  PredictMeanAndCovariance();

  if (debug) {
    cout << "  Predicted state = " << x_ << endl;
    cout << "  Predicted covariance = " << P_ << endl;
  }

  // Predict Measurement
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    if (debug) {
      cout << " Predicting Radar Measurement" << endl;
    }

    PredictMeasurement(meas_package, RADAR_N_Z);
  } else {
    if (debug) {
      cout << " Predicting Lidar Measurement" << endl;
    }

    PredictMeasurement(meas_package, LIDAR_N_Z);
  }

  if (debug) {
    cout << " z_pred_: " << endl << z_pred_ << endl;
    cout << " S_: " << endl << S_ << endl;
  }

  // Update state
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    if (debug) {
      cout << " Updating Radar Measurement" << endl;
    }

    UpdateRadar(meas_package, RADAR_N_Z);
  } else {
    if (debug) {
      cout << " Updating Lidar Measurement" << endl;
    }

    UpdateLidar(meas_package, LIDAR_N_Z);
  }

  if (debug) {
    cout << "  Updated state = " << x_ << endl;
    cout << "  Updated covariance = " << P_ << endl;
  }
}

void UKF::PredictMeasurement(MeasurementPackage meas_package, int n_z) {
  // create matrix for sigma points in measurement space
  Zsig_ = MatrixXd(n_z, 2 * n_aug_ + 1);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    // measurement model
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      double v1 = cos(yaw) * v;
      double v2 = sin(yaw) * v;
      double r = sqrt(p_x * p_x + p_y * p_y);
      Zsig_(0, i) = r;
      Zsig_(1, i) = atan2(p_y, p_x);                                 //phi
      if (r > 0.001)
        Zsig_(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y);  //r_dot
      else
        Zsig_(2, i) = 0;
    } else {
      Zsig_(0, i) = p_x;
      Zsig_(1, i) = p_y;
    }
  }

  //mean predicted measurement
  z_pred_ = VectorXd(n_z);
  z_pred_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred_ = z_pred_ + weights_(i) * Zsig_.col(i);
  }

  //measurement covariance matrix S
  S_ = MatrixXd(n_z, n_z);
  S_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig_.col(i) - z_pred_;

    //angle normalization
    z_diff(1) = NormalizeAngle(z_diff(1));

    S_ = S_ + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    R << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;
  } else {
    R << std_radr_ * std_radr_, 0, 0, 0, std_radphi_ * std_radphi_, 0, 0, 0, std_radrd_
        * std_radrd_;
  }
  S_ = S_ + R;
}

void UKF::UpdateLidar(MeasurementPackage meas_package, int n_z) {
  // Extarct the actual measurement
  VectorXd z = meas_package.raw_measurements_;

  //calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig_.col(i) - z_pred_;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S_.inverse();

  //residual
  VectorXd z_diff = z - z_pred_;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S_ * K.transpose();

  // Calculate NIS
  float nis = z_diff.transpose() * S_.inverse() * z_diff;
  cout << "Lidar NIS : " << nis << endl;
}

void UKF::UpdateRadar(MeasurementPackage meas_package, int n_z) {
  // Extarct the actual measurement
  VectorXd z = meas_package.raw_measurements_;

  //calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig_.col(i) - z_pred_;
    //angle normalization
    z_diff(1) = NormalizeAngle(z_diff(1));

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    x_diff(3) = NormalizeAngle(x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S_.inverse();

  //residual
  VectorXd z_diff = z - z_pred_;

  //angle normalization
  z_diff(1) = NormalizeAngle(z_diff(1));

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S_ * K.transpose();

  // Calculate NIS
  float nis = z_diff.transpose() * S_.inverse() * z_diff;
  cout << "Ridar NIS : " << nis << endl;
}

void UKF::GenerateAugmentedSigmaPoints() {
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.fill(0);
  x_aug.head(n_x_) = x_;

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug_.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug_.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug_.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }
}

void UKF::SigmaPointPrediction(double delta_t) {
  //predict sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    //extract values for better readability
    double p_x = Xsig_aug_(0, i);
    double p_y = Xsig_aug_(1, i);
    double v = Xsig_aug_(2, i);
    double yaw = Xsig_aug_(3, i);
    double yawd = Xsig_aug_(4, i);
    double nu_a = Xsig_aug_(5, i);
    double nu_yawdd = Xsig_aug_(6, i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
      float t = yaw + yawd * delta_t;
      t = NormalizeAngle(t);
      px_p = p_x + v / yawd * (sin(t) - sin(yaw));
      py_p = p_y + v / yawd * (cos(yaw) - cos(t));
    } else {
      px_p = p_x + v * delta_t * cos(yaw);
      py_p = p_y + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;

    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }
}

void UKF::PredictMeanAndCovariance() {

  //create vector for predicted state
  VectorXd x = VectorXd(n_x_);

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);

  //predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x = x + weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x;
    //angle normalization
    x_diff(3) = NormalizeAngle(x_diff(3));

    P = P + weights_(i) * x_diff * x_diff.transpose();
  }

  x_ = x;
  P_ = P;
}

double UKF::NormalizeAngle(double phi) {

  while (phi > M_PI)
    phi -= 2.0 * M_PI;

  while (phi < -M_PI)
    phi += 2.0 * M_PI;

  return phi;
}

