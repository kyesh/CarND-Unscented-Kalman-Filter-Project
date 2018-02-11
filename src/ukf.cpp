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
  P_ = 0.5*MatrixXd::Identity(n_x_,n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 3;
  
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

  Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
std::cout << "Constructor Complete " << std::endl;
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
    cout << "EKF: Not Initilized " << endl;
    
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
	cout << "EKF Initilized" << endl;
    return;
  }
 cout << "Start Prediction" << endl;
  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

	//compute the time elapsed between the current and previous measurements

	float dt = (measurement_pack.timestamp_ - time_us_)/1000000.0; // 1000000.0;	//dt - expressed in seconds

	time_us_ = measurement_pack.timestamp_;

cout << "before calle predict" << endl;
  Prediction(dt);
cout << "End Prediction" << endl;

//cout << "ekf_.x_:" << ekf_.x_ << endl;

cout << "Start Update" << endl;
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	UpdateRadar(measurement_pack);
  } else {
    // Laser updates
	UpdateLidar(measurement_pack);
  }

  // print the output
//  cout << "x_ = " << x_ << endl;
//  cout << "P_ = " << P_ << endl;
cout << "End Update" << endl;
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

  //MatrixXd XsigAug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  AugmentedSigmaPoints();
  cout << "\tAfter AugmentedSigmaPoints" << endl;
  //MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
  SigmaPointPrediction(delta_t);
  cout << "\tAfter SigmaPointPrediction" << endl;
  PredictMeanAndCovariance();
  cout << "\tAfter PredictMeanAndCovariance" << endl;
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
  cout << "\tInside Update Lidar" << endl;
  VectorXd z_out = VectorXd(2);
  MatrixXd S_out = MatrixXd(2, 2);
  MatrixXd Zsig = MatrixXd(2, 2 * n_aug_ + 1);
  cout << "\tBuilt Matrixes" << endl;
  PredictLidarMeasurement(&z_out, &S_out, &Zsig);
  UpdateLidarState(S_out, z_out, meas_package.raw_measurements_, Zsig);
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
  cout << "\tInside Update Radar" << endl;
  VectorXd z_out = VectorXd(3);
  MatrixXd S_out = MatrixXd(3, 3);
  MatrixXd Zsig = MatrixXd(3, 2 * n_aug_ + 1);
  cout << "\tBuilt Matrixes" << endl;
  PredictRadarMeasurement(&z_out, &S_out, &Zsig);
  cout << "\tBefore z" << endl;
  VectorXd z = meas_package.raw_measurements_.head(3);
cout << "\tBefore UpdateRadarState" << endl;
  UpdateRadarState(S_out, z_out, z, Zsig);
}

/*******************************************************************************
* Programming assignment functions: 
*******************************************************************************/

void UKF::AugmentedSigmaPoints() {

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

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
}

/*******************************************************************************
* Programming assignment functions: 
*******************************************************************************/

void UKF::SigmaPointPrediction(double delta_t) {
  //create matrix with predicted sigma points as columns
  Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

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

}

void UKF::PredictMeanAndCovariance() {
cout << "\t\tInside PredictMeanAndCovariance" << endl;
/*******************************************************************************
 * Student part begin
 ******************************************************************************/


  //predict state mean
  for(int i = 0; i < n_x_; i++){
    x_(i) = Xsig_pred.row(i).dot(weights_);
  }
  cout << "\t\tcovariance matrix" << endl;
  //predict state covariance matrix
  P_.fill(0.0);
  cout << "\t\tAfter Fill" << endl;
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    cout << "\t\t\tBefor x_diff: " << i << endl;
    VectorXd x_diff = Xsig_pred.col(i) - x_;
      //angle normalization
    cout << "\t\t\tangle normalization: " << i << endl;
    x_diff(3) = fixThetaRange(x_diff(3));
    //while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    //while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    cout << "\t\t\tBefor P Sum: " << i << endl;
  P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
/*******************************************************************************
 * Student part end
 ******************************************************************************/

}

/*******************************************************************************
* Programming assignment functions: 
*******************************************************************************/

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out) {

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/
  cout << "Before Transform" << endl;
  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred(0,i);
    double p_y = Xsig_pred(1,i);
    double v  = Xsig_pred(2,i);
    double yaw = Xsig_pred(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }
cout << "Before mean predict" << endl; 
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
cout << "Before S" << endl;
  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
cout << "Before R" << endl;
  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
cout << "After R" << endl;
  S = S + R;

  
/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  //std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  //std::cout << "S: " << std::endl << S << std::endl;

cout << "Before Saving" << endl;
  //write result
  *z_out = z_pred;
  cout << "Z Saved" << endl;
  *S_out = S;
  cout << "S Saved" << endl;
  *Zsig_out = Zsig;
  cout << "Zsig Saved" << endl;
}

/*******************************************************************************
* Programming assignment functions: 
*******************************************************************************/

void UKF::UpdateRadarState(MatrixXd S, MatrixXd z_pred, VectorXd z, MatrixXd Zsig) {
  cout << "In Update Radar State" << endl;
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    z_diff(1) = fixThetaRange(z_diff(1));
    //while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    //while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x_;
    //angle normalization
    x_diff(3) = fixThetaRange(x_diff(3));
    //while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    //while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  cout << "Before K" << endl;
  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  cout << "Before Assingment" << endl;
  x_ = x_ + K * z_diff;
  cout << "Before P_ Assingment" << endl;
  P_ = P_ - K*S*K.transpose();

/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  //std::cout << "Updated state x: " << std::endl << x << std::endl;
  //std::cout << "Updated state covariance P: " << std::endl << P << std::endl;

}

/*******************************************************************************
* Programming assignment functions: 
*******************************************************************************/

void UKF::PredictLidarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out) {

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 2;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred(0,i);
    double p_y = Xsig_pred(1,i);
    double v  = Xsig_pred(2,i);
    double yaw = Xsig_pred(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = p_x;                       
    Zsig(1,i) = p_y;                       
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_laspx_*std_laspx_, 0,
          0, std_laspy_*std_laspy_;
  S = S + R;

  
/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  //std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  //std::cout << "S: " << std::endl << S << std::endl;

  //write result
  *z_out = z_pred;
  *S_out = S;
  *Zsig_out = Zsig;
}

/*******************************************************************************
* Programming assignment functions: 
*******************************************************************************/

void UKF::UpdateLidarState(MatrixXd S, MatrixXd z_pred, VectorXd z, MatrixXd Zsig) {

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 2;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x_;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  //std::cout << "Updated state x: " << std::endl << x << std::endl;
  //std::cout << "Updated state covariance P: " << std::endl << P << std::endl;

}

//Forces range between - pi and pi
double UKF::fixThetaRange(double theta){

	theta = fmod(theta,2*M_PI);//Brings between -2pi and pi
	if(theta > M_PI){//convert to negative pi if greater than pi
		theta =  theta - 2*M_PI;
	}else if(theta < -M_PI){
		theta = 2*M_PI - theta;
	}

	return theta;

}
