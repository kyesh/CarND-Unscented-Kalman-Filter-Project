#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {

  n_x_ = 5;
  n_aug_ = n_x_ + 2;
  lambda_ = 3 - n_aug_;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  //create vector for weights
  weights_ = VectorXd(2*n_aug_+1);

  //set weights
  weights_.fill(1/(2*(lambda_ + n_aug_)));
  weights_(0) = (lambda_ /(lambda_ +n_aug_));
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    x_ = VectorXd(n_x_); 
    double iv = 0;
    double iPsi = 0;
    double iPsiDot = 0;
    /**
    
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    //cout << "EKF: Not Initilized " << endl;
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
	//cout << "Radar!" << endl;
	float x,y;
	//y = sin(phi)*rho;
	y = sin(measurement_pack.raw_measurements_[1])*measurement_pack.raw_measurements_[0];
	//x = cos(phi)*rho;
	x = cos(measurement_pack.raw_measurements_[1])*measurement_pack.raw_measurements_[0];	
	//cout << "x: " << x << " y: " << y << endl;
	x_ << x,y,iv,iPsi,iPsiDot;
	//cout << "ekf_.x_:" << ekf_.x_ << endl;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
	//cout << "Laser!" << endl;
		//set the state with the initial location and zero velocity

		x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], iv,iPsi,iPsiDot;






    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
	time_us_ = measurement_pack.timestamp_;
	//cout << "EKF Initilized" << endl;
    return;
  }
 //cout << "Start Prediction" << endl;
  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
	//compute the time elapsed between the current and previous measurements

	float dt = (measurement_pack.timestamp_ - time_us_)/1000000.0; // 1000000.0;	//dt - expressed in seconds

	time_us_ = measurement_pack.timestamp_;



//	float dt_2 = dt * dt;

//	float dt_3 = dt_2 * dt;

//	float dt_4 = dt_3 * dt;



	//Modify the F matrix so that the time is integrated
 //cout << "before setting F" << endl;
	//F_ = MatrixXd(n_x_, n_x_);

//	F_ = << 1, 0, 0, 0, 0,
//	        0, 1, 0, 0, 0,
//	        0, 0, 1, 0, 0,
//	        0, 0, 0, 1, 0,
//	        0, 0, 0, 0, 1;

	//ekf_.F_(0, 2) = dt;

	//ekf_.F_(1, 3) = dt;



	//set the process covariance matrix Q
//cout << "before initilizging Q" << endl;
//	ekf_.Q_ = MatrixXd(4, 4);
//cout << "before setting Q" << endl;
//	ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,

//			   0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,

//			   dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,

//			   0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
//cout << "before calle predict" << endl;
  Prediction(dt);
//cout << "End Prediction" << endl;

//cout << "ekf_.x_:" << ekf_.x_ << endl;

//cout << "Start Update" << endl;
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	//cout << "Radar!" << endl;
	//Hj_ = tools.CalculateJacobian(ekf_.x_);
        //ekf_.H_ = Hj_;
        //ekf_.R_ = R_radar_;

	//ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
	//cout << "Laser!" << endl;
	//ekf_.H_ = H_laser_;
	//ekf_.R_ = R_laser_;

	//ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
 //cout << "End Update" << endl;
 //cout << "ekf_.x_:" << ekf_.x_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  MatrixXd XsigAug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  AugmentedSigmaPoints(&XsigAug);
  
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
  SigmaPointPrediction(&Xsig_pred, XsigAug, delta_t);

  PredictMeanAndCovariance(Xsig_pred);
  
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}

/*******************************************************************************
* Programming assignment functions: 
*******************************************************************************/

void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/
 
  //create augmented mean state
  x_aug << x_,0,0;
  MatrixXd meanExtended = MatrixXd(n_aug_,n_aug_);
  meanExtended << x_aug,x_aug,x_aug,x_aug,x_aug,x_aug,x_aug; //Should really loop n_x times
  //create augmented covariance matrix
  MatrixXd nu = MatrixXd(2,2);
  nu << std_a_*std_a_,0,
                    0, std_yawdd_*std_yawdd_;
  P_aug << P_ , MatrixXd::Zero(n_x_,2),
           MatrixXd::Zero(2,n_x_), nu;
  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();
  //create augmented sigma points
  Xsig_aug << x_aug, meanExtended + sqrt(lambda_+n_aug_)*A, meanExtended - sqrt(lambda_+n_aug_)*A;
/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  //std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;

  //write result
  *Xsig_out = Xsig_aug;

}

/*******************************************************************************
* Programming assignment functions: 
*******************************************************************************/

void UKF::SigmaPointPrediction(MatrixXd* Xsig_out, MatrixXd Xsig_aug, double delta_t) {
  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  //predict sigma points
  //avoid division by zero
  //write predicted sigma points into right column
  for(int i = 0; i < (2 * n_aug_ + 1); i++){
      
      double phi = Xsig_aug(3,i);
      double phidot = Xsig_aug(4,i); 
      double vk = Xsig_aug(2,i);
      double cosPhi = cos(Xsig_aug(3,i));
      double sinPhi = sin(Xsig_aug(3,i));
      double t22 = delta_t*delta_t/2;
      double va = Xsig_aug(5,i);
      double vPhidotdot = Xsig_aug(6,i);;
      
      if(fabs(Xsig_aug(4,i)) > .0001){
          Xsig_pred(0,i) = Xsig_aug(0,i) + vk/phidot*(sin(phi + phidot*delta_t) - sin(phi)) + 0.5*va*delta_t*delta_t*cos(Xsig_aug(3,i));
          Xsig_pred(1,i) = Xsig_aug(1,i) + vk/phidot*(-cos(phi + phidot*delta_t) + cos(phi)) + t22*sinPhi*va;
          Xsig_pred(2,i) = Xsig_aug(2,i) + 0 + delta_t*va;
          Xsig_pred(3,i) = Xsig_aug(3,i) + phidot*delta_t + t22*vPhidotdot;
          Xsig_pred(4,i) = Xsig_aug(4,i) + 0 + delta_t*vPhidotdot;  
      }else{
          Xsig_pred(0,i) = Xsig_aug(0,i) + vk*cosPhi*delta_t + t22*cosPhi*va;
          Xsig_pred(1,i) = Xsig_aug(1,i) + vk*sinPhi*delta_t + t22*sinPhi*va;
          Xsig_pred(2,i) = Xsig_aug(2,i) + 0 + delta_t*va;
          Xsig_pred(3,i) = Xsig_aug(3,i) + phidot*delta_t + t22*vPhidotdot;
          Xsig_pred(4,i) = Xsig_aug(4,i) + 0 + delta_t*vPhidotdot;
          
      }
      
      
  }
  

/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //write result
  *Xsig_out = Xsig_pred;

}

void UKF::PredictMeanAndCovariance(MatrixXd Xsig_pred) {

/*******************************************************************************
 * Student part begin
 ******************************************************************************/


  //predict state mean
  for(int i = 0; i < n_x_; i++){
    x_(i) = Xsig_pred.row(i).dot(weights_);
  }
  //predict state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
  VectorXd x_diff = Xsig_pred.col(i) - x_;
      //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
  P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
/*******************************************************************************
 * Student part end
 ******************************************************************************/

}
