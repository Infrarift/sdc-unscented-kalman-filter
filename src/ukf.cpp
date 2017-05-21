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
UKF::UKF()
{
	use_laser_ = true;
	use_radar_ = true;

	sigma_manager_ = SigmaPointManager();

	previous_time_stamp_ = 0;
	delta_time_ = 0;

	x_ = VectorXd(5);
	x_.fill(0);

	P_ = MatrixXd(5, 5);
	P_.setIdentity();
	P_ = P_ * 1;

	std_a_ = 6;
	std_yawdd_ = 1.5;

	std_laspx_ = 0.15;
	std_laspy_ = 0.15;

	std_radr_ = 0.3;
	std_radphi_ = 0.03;
	std_radrd_ = 0.3;

	n_x_ = 5;
	n_aug_ = 7;

	lambda_ = 3 - n_aug_;
	weights_ = sigma_manager_.GeneratePredictionWeights(lambda_);
}

UKF::~UKF()
{
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
		UpdateRadar(meas_package);

	if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
		UpdateLidar(meas_package);
}


void UKF::Prediction(double delta_t)
{
	auto augmented_sigma_points = MatrixXd(7, 15);
	auto predicted_sigma_points = MatrixXd(5, 15);
	sigma_manager_.GenerateAugmentedSigmaPoints(x_, P_, n_aug_, std_a_, std_yawdd_, lambda_, &augmented_sigma_points);
	sigma_manager_.PredictSigmaPoints(augmented_sigma_points, delta_t, &predicted_sigma_points);
	sigma_manager_.PredictedMeanFromSigma(predicted_sigma_points, &x_, lambda_);
	sigma_manager_.PredictedCovarianceFromSigma(predicted_sigma_points, x_, &P_, lambda_);
	Xsig_pred_ = predicted_sigma_points;	
}


void UKF::UpdateLidar(MeasurementPackage meas_package)
{
	UpdateDeltaTime(meas_package.timestamp_);
	Prediction(delta_time_);
	auto predicted_m_sigma = MatrixXd(2, 15);
	auto predicted_m_mean = VectorXd(2);
	auto predicted_m_covariance = MatrixXd(2, 2);
	PredictionAsLaser(Xsig_pred_, &predicted_m_sigma, &predicted_m_mean, &predicted_m_covariance);
	UpdateStateWithLaser(meas_package, Xsig_pred_, predicted_m_sigma, predicted_m_mean, predicted_m_covariance, &x_, &P_);
}

void UKF::UpdateRadar(MeasurementPackage meas_package)
{
	UpdateDeltaTime(meas_package.timestamp_);
	Prediction(delta_time_);
	auto predicted_m_sigma = MatrixXd(3, 15);
	auto predicted_m_mean = VectorXd(3);
	auto predicted_m_covariance = MatrixXd(3, 3);
	PredictionAsRadar(Xsig_pred_, &predicted_m_sigma, &predicted_m_mean, &predicted_m_covariance);
	UpdateStateWithRadar(meas_package, Xsig_pred_, predicted_m_sigma, predicted_m_mean, predicted_m_covariance, &x_, &P_);
}

void UKF::PredictionAsLaser(MatrixXd& predicted_sigma_points, MatrixXd* predicted_m_sigma, VectorXd* prediected_mean, MatrixXd* predicted_covariance)
{
	auto n_z = 2;
	MatrixXd m_sigma_points = MatrixXd(n_z, 15);

	for (int i = 0; i < sigma_manager_.NumOfSigmaPointsFromDimensions(n_aug_); i++) {  //2n+1 simga points

		double p_x = predicted_sigma_points(0, i);
		double p_y = predicted_sigma_points(1, i);

		m_sigma_points(0, i) = p_x;
		m_sigma_points(1, i) = p_y;
	}

	//mean predicted measurement
	VectorXd z_pred = VectorXd(n_z);
	z_pred.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++)
		z_pred = z_pred + weights_(i) * m_sigma_points.col(i);

	//measurement covariance matrix S
	MatrixXd S = MatrixXd(n_z, n_z);
	S.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++)
	{
		VectorXd z_diff = m_sigma_points.col(i) - z_pred;
		S = S + weights_(i) * z_diff * z_diff.transpose();
	}

	//add measurement noise covariance matrix
	MatrixXd R = MatrixXd(n_z, n_z);
	R <<
		std_laspx_*std_laspx_, 0,
		0, std_laspy_*std_laspy_;
	S = S + R;

	*predicted_m_sigma = m_sigma_points;
	*prediected_mean = z_pred;
	*predicted_covariance = S;
}

void UKF::UpdateStateWithLaser(MeasurementPackage meas_package, MatrixXd& predicted_state_sigma, MatrixXd& predicted_m_sigma, VectorXd& predicted_m_mean, MatrixXd& predicted_m_covariance, VectorXd* state_mean, MatrixXd* state_covariance)
{
	auto cross_correlation = MatrixXd(5, 2);
	cross_correlation.fill(0.0);

	for (int i = 0, m = predicted_state_sigma.cols(); i < m; i++)
	{
		VectorXd state_diff = predicted_state_sigma.col(i) - *state_mean;
		VectorXd measurement_diff = predicted_m_sigma.col(i) - predicted_m_mean;
		cross_correlation = cross_correlation + weights_(i) * state_diff * measurement_diff.transpose();
	}

	MatrixXd kalman_gain = cross_correlation * predicted_m_covariance.inverse();
	VectorXd residual = meas_package.raw_measurements_ - predicted_m_mean;

	*state_mean = *state_mean + kalman_gain * residual;
	*state_covariance = *state_covariance - kalman_gain * predicted_m_covariance * kalman_gain.transpose();
}

void UKF::PredictionAsRadar(MatrixXd& predicted_sigma_points, MatrixXd* predicted_m_sigma, VectorXd* prediected_mean, MatrixXd* predicted_covariance)
{

	MatrixXd m_sigma_points = MatrixXd(3, 15);
	auto n_z = 3;

	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

											   // extract values for better readibility
		double p_x = predicted_sigma_points(0, i);
		double p_y = predicted_sigma_points(1, i);
		double v = predicted_sigma_points(2, i);
		double yaw = predicted_sigma_points(3, i);

		double v1 = cos(yaw)*v;
		double v2 = sin(yaw)*v;

		// measurement model
		auto px_py_sq = sqrt(p_x*p_x + p_y*p_y);
		m_sigma_points(0, i) = px_py_sq;                        //r
		m_sigma_points(1, i) = atan2(p_y, p_x);                                 //phi
		m_sigma_points(2, i) = px_py_sq == 0 ? 0 : (p_x*v1 + p_y*v2) / px_py_sq;   //r_dot
	}

	//mean predicted measurement
	VectorXd z_pred = VectorXd(n_z);
	z_pred.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++)
		z_pred = z_pred + weights_(i) * m_sigma_points.col(i);

	//measurement covariance matrix S
	MatrixXd S = MatrixXd(n_z, n_z);
	S.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) 
	{ 
		VectorXd z_diff = m_sigma_points.col(i) - z_pred;
		z_diff(1) = Tools::NormalizeAngleAroundPi(z_diff(1));
		S = S + weights_(i) * z_diff * z_diff.transpose();
	}

	//add measurement noise covariance matrix
	MatrixXd R = MatrixXd(n_z, n_z);
	R << 
		std_radr_*std_radr_, 0, 0,
		0, std_radphi_*std_radphi_, 0,
		0, 0, std_radrd_*std_radrd_;
	S = S + R;

	*predicted_m_sigma = m_sigma_points;
	*prediected_mean = z_pred;
	*predicted_covariance = S;
}

void UKF::UpdateStateWithRadar(MeasurementPackage meas_package, MatrixXd& predicted_state_sigma, MatrixXd& predicted_m_sigma, VectorXd& predicted_m_mean, MatrixXd& predicted_m_covariance, VectorXd* state_mean, MatrixXd* state_covariance)
{
	auto cross_correlation = MatrixXd(5, 3);
	cross_correlation.fill(0.0);

	for (int i = 0, m = predicted_state_sigma.cols(); i < m; i++)
	{
		VectorXd state_diff = predicted_state_sigma.col(i) - *state_mean;
		state_diff(3) = Tools::NormalizeAngleAroundPi(state_diff(3));

		VectorXd measurement_diff = predicted_m_sigma.col(i) - predicted_m_mean;
		measurement_diff(1) = Tools::NormalizeAngleAroundPi(measurement_diff(1));

		cross_correlation = cross_correlation + weights_(i) * state_diff * measurement_diff.transpose();
	}

	MatrixXd kalman_gain = cross_correlation * predicted_m_covariance.inverse();
	VectorXd residual = meas_package.raw_measurements_ - predicted_m_mean;
	residual(1) = Tools::NormalizeAngleAroundPi(residual(1));

	std::cout << std::endl << residual << std::endl;

	*state_mean = *state_mean + kalman_gain * residual;
	*state_covariance = *state_covariance - kalman_gain * predicted_m_covariance * kalman_gain.transpose();
}

void UKF::UpdateDeltaTime(double time_stamp)
{
	if (previous_time_stamp_ == 0)
	{
		previous_time_stamp_ = time_stamp;
		return;
	}
	
	delta_time_ = (time_stamp - previous_time_stamp_) / 1000000.0;
	previous_time_stamp_ = time_stamp;
}
