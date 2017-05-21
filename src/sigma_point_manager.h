#ifndef SIGMA_POINT_MANAGER_H
#define SIGMA_POINT_MANAGER_H

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class SigmaPointManager
{
public:
	SigmaPointManager();
	~SigmaPointManager();
	static int NumOfSigmaPointsFromDimensions(int dimensions);
	static VectorXd* CreateAugmentedStateVector(VectorXd& state_vector, VectorXd& output_vector);
	static MatrixXd* CreateAugmentedCovarianceMatrix(MatrixXd& covariance_matrix, double std_acceleration, double std_yaw_acceleration, MatrixXd& output_matrix);
	static MatrixXd* GenerateAugmentedSigmaPoints(VectorXd& state_vector, MatrixXd& covariance_matrix, int aug_n, double std_acceleration, double std_yaw_acceleration, double lambda, MatrixXd* output_matrix);
	static MatrixXd* PredictSigmaPoints(MatrixXd& augmented_sigma_points, double delta_time_in_seconds, MatrixXd* output_sigma_points);
	static VectorXd* PredictedMeanFromSigma(MatrixXd& predicted_sigma_points, VectorXd* output_vector, double lambda);
	static MatrixXd* PredictedCovarianceFromSigma(MatrixXd& predicted_sigma_points, VectorXd& predicted_mean, MatrixXd* output_matrix, double lambda);
	static VectorXd GeneratePredictionWeights(double lambda);
};

#endif
