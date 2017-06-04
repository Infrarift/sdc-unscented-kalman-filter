#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

UKF::UKF()
{
	use_laser_ = true;
	use_radar_ = true;

	sigma_manager_ = SigmaPointManager();

	previous_time_stamp_ = 0;
	delta_time_ = 0;

	n_x_ = 5;
	n_aug_ = 7;
	lambda_ = 3 - n_aug_;
	n_sigma_ = sigma_manager_.NumOfSigmaPointsFromDimensions(n_aug_);
	weights_ = sigma_manager_.GeneratePredictionWeights(n_aug_, lambda_);

	std_a_ = 3.5;
	std_yawdd_ = 1.2;

	state_entity_ = Entity(n_x_, n_sigma_);
	state_entity_.AddNormalizationIndex(3);

	double laser_noise[] = { 0.15, 0.15 };
	laser_entity_ = Entity(2, n_sigma_, laser_noise);

	double radar_noise[] = { 0.3, 0.03, 0.3 };
	radar_entity_ = Entity(3, n_sigma_, radar_noise);
	radar_entity_.AddNormalizationIndex(1);
}

UKF::~UKF()
{
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
		ProcessWithSensor(meas_package, radar_entity_, radar_transformer_);

	if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
		ProcessWithSensor(meas_package, laser_entity_, laser_transformer_);

	x_ = state_entity_.mean_;
	P_ = state_entity_.covariance_;
	if (!radar_entity_.NIS_.empty()) NIS_radar_ = radar_entity_.NIS_.back();
	if (!laser_entity_.NIS_.empty()) NIS_laser_ = laser_entity_.NIS_.back();
}

void UKF::ProcessWithSensor(MeasurementPackage meas_package, Entity& sensor, Transformer& transformer)
{
	UpdateDeltaTime(meas_package.timestamp_);
	PredictState(delta_time_);
	ConvertPredictionToSensorSpace(state_entity_, &sensor, transformer);
	CalculateNIS(meas_package, sensor);
	UpdateStateWithSensor(meas_package, state_entity_, sensor);
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

void UKF::PredictState(double delta_t)
{
	auto augmented_sigma_points = MatrixXd(n_aug_, n_sigma_);
	sigma_manager_.GenerateAugmentedSigmaPoints(state_entity_, n_aug_, std_a_, std_yawdd_, lambda_, &augmented_sigma_points);
	sigma_manager_.PredictSigmaPoints(augmented_sigma_points, delta_t, state_entity_, weights_);
}

void UKF::ConvertPredictionToSensorSpace(Entity& state, Entity* sensor, Transformer& transformer)
{
	sensor->Reset();
	transformer.Convert(state, *sensor);
	PredictSensorMean(sensor);
	PredictSensorCovariance(sensor);
}

void UKF::PredictSensorCovariance(Entity* sensor)
{
	sensor->covariance_.fill(0.0);
	for (auto i = 0; i < sensor->n_sigma_; i++)
	{
		VectorXd z_diff = sensor->sigma_points_.col(i) - sensor->mean_;
		sensor->covariance_ = sensor->covariance_ + weights_(i) * z_diff * z_diff.transpose();
	}
	sensor->covariance_ = sensor->covariance_ + sensor->GetNoiseMatrix();
}

void UKF::PredictSensorMean(Entity* sensor)
{
	for (auto i = 0; i < sensor->n_sigma_; i++)
		sensor->mean_ = sensor->mean_ + weights_(i) * sensor->sigma_points_.col(i);
}

void UKF::UpdateStateWithSensor(MeasurementPackage& meas_package, Entity& state, Entity& sensor)
{
	auto cross_correlation = CalculateCrossCorrelation(state, sensor);
	MatrixXd kalman_gain = cross_correlation * sensor.covariance_.inverse();
	VectorXd residual = meas_package.raw_measurements_ - sensor.mean_;
	sensor.NormalizeVector(residual);

	state.mean_ = state.mean_ + kalman_gain * residual;
	state.covariance_ = state.covariance_ - kalman_gain * sensor.covariance_ * kalman_gain.transpose();
}

void UKF::CalculateNIS(MeasurementPackage& meas_package, Entity& sensor)
{
	VectorXd residual = meas_package.raw_measurements_ - sensor.mean_;
	sensor.NormalizeVector(residual);
	double NIS = residual.transpose() * sensor.covariance_.inverse() * residual;
	sensor.NIS_.push_back(NIS);
}

MatrixXd UKF::CalculateCrossCorrelation(Entity& state, Entity& sensor)
{
	auto cross_correlation = MatrixXd(state.dimensions_, sensor.dimensions_);
	cross_correlation.fill(0.0);
	for (auto i = 0, m = sensor.n_sigma_; i < m; i++)
	{
		VectorXd state_diff = state.sigma_points_.col(i) - state.mean_;
		VectorXd measurement_diff = sensor.sigma_points_.col(i) - sensor.mean_;
		state.NormalizeVector(state_diff);
		sensor.NormalizeVector(measurement_diff);
		cross_correlation = cross_correlation + weights_(i) * state_diff * measurement_diff.transpose();
	}
	return cross_correlation;
}

void UKF::PrintNIS()
{
	std::cout << "LASER NIS --- ";
	laser_entity_.PrintNIS();

	std::cout << "RADAR NIS --- ";
	radar_entity_.PrintNIS();
}

