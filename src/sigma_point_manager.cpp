#include "sigma_point_manager.h"
#include "Eigen/Dense"
#include <vector>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

SigmaPointManager::SigmaPointManager()
{
}

SigmaPointManager::~SigmaPointManager()
{
}

int SigmaPointManager::NumOfSigmaPointsFromDimensions(int dimensions)
{
	return 1 + 2 * dimensions;
}

MatrixXd* SigmaPointManager::GenerateAugmentedSigmaPoints(Entity& state, int aug_n, double std_acceleration, double std_yaw_acceleration, double lambda, MatrixXd* output_matrix)
{
	auto augmented_state = VectorXd(aug_n);
	CreateAugmentedStateVector(state.mean_, augmented_state);

	auto augmented_covariance = MatrixXd(aug_n, aug_n);
	CreateAugmentedCovarianceMatrix(state.covariance_, std_acceleration, std_yaw_acceleration, augmented_covariance);

	auto lambda_aug_sqrt = sqrt(lambda + aug_n);
	MatrixXd square_root_matrix = augmented_covariance.llt().matrixL();

	auto aug_sigma_points = MatrixXd(aug_n, state.n_sigma_);
	aug_sigma_points.col(0) = augmented_state;

	for (auto i = 0; i < aug_n; i++)
	{
		auto sigma_pos = lambda_aug_sqrt * square_root_matrix.col(i);
		auto first_index = i + 1;
		auto second_index = first_index + aug_n;
		aug_sigma_points.col(first_index) = augmented_state + sigma_pos;
		aug_sigma_points.col(second_index) = augmented_state - sigma_pos;
	}

	*output_matrix = aug_sigma_points;
	return output_matrix;
}

VectorXd* SigmaPointManager::CreateAugmentedStateVector(VectorXd& state_vector, VectorXd& output_vector)
{
	output_vector.head(5) = state_vector;
	output_vector(5) = 0;
	output_vector(6) = 0;
	return &output_vector;
}

MatrixXd* SigmaPointManager::CreateAugmentedCovarianceMatrix(MatrixXd& covariance_matrix, double std_acceleration, double std_yaw_acceleration, MatrixXd& output_matrix)
{
	output_matrix.fill(0.0);
	output_matrix.topLeftCorner(5, 5) = covariance_matrix;
	output_matrix(5, 5) = std_acceleration * std_acceleration;
	output_matrix(6, 6) = std_yaw_acceleration * std_yaw_acceleration;
	return &output_matrix;
}

void SigmaPointManager::PredictSigmaPoints(MatrixXd& augmented_sigma_points, double delta_time_in_seconds, Entity& state, VectorXd& weights)
{
	for (auto i = 0; i < state.n_sigma_; i++)
	{
		double p_x = augmented_sigma_points(0, i);
		double p_y = augmented_sigma_points(1, i);
		double v = augmented_sigma_points(2, i);
		double yaw = augmented_sigma_points(3, i);
		double yawd = augmented_sigma_points(4, i);
		double nu_a = augmented_sigma_points(5, i);
		double nu_yawdd = augmented_sigma_points(6, i);
		double px_p, py_p;

		if (fabs(yawd) > 0.001)
		{
			px_p = p_x + v / yawd * (sin(yaw + yawd * delta_time_in_seconds) - sin(yaw));
			py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_time_in_seconds));
		}
		else
		{
			px_p = p_x + v * delta_time_in_seconds * cos(yaw);
			py_p = p_y + v * delta_time_in_seconds * sin(yaw);
		}

		double v_p = v;
		double yaw_p = yaw + yawd * delta_time_in_seconds;
		double yawd_p = yawd;

		px_p = px_p + 0.5 * nu_a * delta_time_in_seconds * delta_time_in_seconds * cos(yaw);
		py_p = py_p + 0.5 * nu_a * delta_time_in_seconds * delta_time_in_seconds * sin(yaw);
		v_p = v_p + nu_a * delta_time_in_seconds;

		yaw_p = yaw_p + 0.5 * nu_yawdd * delta_time_in_seconds * delta_time_in_seconds;
		yawd_p = yawd_p + nu_yawdd * delta_time_in_seconds;

		state.sigma_points_(0, i) = px_p;
		state.sigma_points_(1, i) = py_p;
		state.sigma_points_(2, i) = v_p;
		state.sigma_points_(3, i) = yaw_p;
		state.sigma_points_(4, i) = yawd_p;
	}

	PredictedMeanFromSigma(state, weights);
	PredictedCovarianceFromSigma(state, weights);
}

void SigmaPointManager::PredictedMeanFromSigma(Entity& state, VectorXd& weights)
{
	state.mean_.fill(0);
	for (auto i = 0, m = state.n_sigma_; i < m; i++)
		state.mean_ = state.mean_ + weights(i) * state.sigma_points_.col(i);
}

void SigmaPointManager::PredictedCovarianceFromSigma(Entity& state, VectorXd& weights)
{
	state.covariance_.fill(0.0);
	for (auto i = 0, m = state.n_sigma_; i < m; i++)
	{
		VectorXd state_difference = state.sigma_points_.col(i) - state.mean_;
		state.NormalizeVector(state_difference);
		state.covariance_ += weights(i) * state_difference * state_difference.transpose();
	}
}

VectorXd SigmaPointManager::GeneratePredictionWeights(int n, double lambda)
{
	auto weights = VectorXd(NumOfSigmaPointsFromDimensions(n));
	auto lambda_plus_n = lambda + n;
	weights.fill(0.5 / lambda_plus_n);
	weights(0) = lambda / lambda_plus_n;
	return weights;
}
