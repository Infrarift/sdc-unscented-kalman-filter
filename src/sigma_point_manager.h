#ifndef SIGMA_POINT_MANAGER_H
#define SIGMA_POINT_MANAGER_H

#include "entity.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class SigmaPointManager
{
public:
	SigmaPointManager();
	~SigmaPointManager();
	static int NumOfSigmaPointsFromDimensions(int dimensions);
	static MatrixXd* GenerateAugmentedSigmaPoints(Entity& state, int aug_n, double std_acceleration, double std_yaw_acceleration, double lambda, MatrixXd* output_matrix);
	static void PredictSigmaPoints(MatrixXd& augmented_sigma_points, double delta_time_in_seconds, Entity& state, VectorXd& weights);
	static VectorXd GeneratePredictionWeights(int n, double lambda);

private:
	static void PredictedMeanFromSigma(Entity& state, VectorXd& weights);
	static void PredictedCovarianceFromSigma(Entity& state, VectorXd& weights);
	static VectorXd* CreateAugmentedStateVector(VectorXd& state_vector, VectorXd& output_vector);
	static MatrixXd* CreateAugmentedCovarianceMatrix(MatrixXd& covariance_matrix, double std_acceleration, double std_yaw_acceleration, MatrixXd& output_matrix);
};

#endif
