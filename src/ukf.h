#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"
#include "sigma_point_manager.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF
{
public:

	SigmaPointManager sigma_manager_;

	bool is_initialized_;
	bool use_laser_;
	bool use_radar_;

	double previous_time_stamp_;
	double delta_time_;

	///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
	VectorXd x_;

	///* state covariance matrix
	MatrixXd P_;

	///* predicted sigma points matrix
	MatrixXd Xsig_pred_;

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
	double std_radrd_;

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

	UKF();
	virtual ~UKF();
	void ProcessMeasurement(MeasurementPackage meas_package);
	void Prediction(double delta_t);
	void UpdateLidar(MeasurementPackage meas_package);
	void UpdateRadar(MeasurementPackage meas_package);
	void PredictionAsRadar(MatrixXd& predicted_sigma_points, MatrixXd* predicted_m_sigma, VectorXd* prediected_mean, MatrixXd* predicted_covariance);
	void UpdateStateWithRadar(MeasurementPackage meas_package, MatrixXd& predicted_state_sigma, MatrixXd& predicted_m_sigma, VectorXd& predicted_m_mean, MatrixXd& predicted_m_covariance, VectorXd* state_mean, MatrixXd* state_covariance);
	void PredictionAsLaser(MatrixXd& predicted_sigma_points, MatrixXd* predicted_m_sigma, VectorXd* prediected_mean, MatrixXd* predicted_covariance);
	void UpdateStateWithLaser(MeasurementPackage meas_package, MatrixXd& predicted_state_sigma, MatrixXd& predicted_m_sigma, VectorXd& predicted_m_mean, MatrixXd& predicted_m_covariance, VectorXd* state_mean, MatrixXd* state_covariance);
	private:
	void UpdateDeltaTime(double time_stamp);
};

#endif
