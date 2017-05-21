#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}
Tools::~Tools() {}

bool Tools::AreInputsValid(const vector<VectorXd>& estimations, const vector<VectorXd>& ground_truth)
{
	if (estimations.size() == 0)
		return false;

	if (estimations.size() != ground_truth.size())
		return false;

	return true;
}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

	VectorXd rmse(4);
	rmse.fill(0);

	if (!AreInputsValid(estimations, ground_truth))
		return rmse;

	for (unsigned int i = 0; i < estimations.size(); ++i)
	{
		VectorXd residual = estimations[i] - ground_truth[i];
		residual = residual.array() * residual.array();
		auto size = residual.size();
		rmse += residual;
	}

	rmse = rmse / estimations.size();
	return rmse;
}

double Tools::NormalizeAngleAroundPi(double input_angle)
{
	// TODO: Optimize this part.
	auto new_angle = atan2(sin(input_angle), cos(input_angle));
	return new_angle;
}
