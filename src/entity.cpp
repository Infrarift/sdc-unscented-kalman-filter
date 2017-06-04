#include "entity.h"
#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;

Entity::Entity()
{
	Initialize(1, 1);
}

Entity::Entity(int dimensions, int n_sigma)
{
	Initialize(dimensions, n_sigma);
}

Entity::Entity(int dimensions, int n_sigma, double noise[])
{
	Initialize(dimensions, n_sigma);
	SetNoise(noise);
}

Entity::~Entity()
{
}

void Entity::Initialize(int dimensions, int n_sigma)
{
	dimensions_ = dimensions;
	n_sigma_ = n_sigma;

	mean_ = VectorXd(dimensions_);
	covariance_ = MatrixXd(dimensions_, dimensions_);
	sigma_points_ = MatrixXd(dimensions_, n_sigma);

	Reset();
}

void Entity::SetNoise(double noise[])
{
	noise_vector_ = VectorXd(dimensions_);
	noise_matrix_ = MatrixXd(dimensions_, dimensions_);
	noise_matrix_.fill(0);

	for (auto i = 0; i < dimensions_; ++i)
		noise_vector_(i) = noise[i];

	for(auto i = 0; i < dimensions_ ; i++)
		noise_matrix_(i, i) = noise_vector_(i) * noise_vector_(i);
}

MatrixXd Entity::GetNoiseMatrix()
{
	return noise_matrix_;
}

VectorXd Entity::GetNoiseVector()
{
	return noise_vector_;
}

void Entity::NormalizeVector(VectorXd& vector)
{
	for (auto i : normalize_index_)
		vector(i) = Tools::NormalizeAngleAroundPi(vector(i));
}

void Entity::NormalizeAngles()
{
	NormalizeVector(mean_);
}

void Entity::AddNormalizationIndex(int index)
{
	normalize_index_.push_back(index);
}

void Entity::Reset()
{
	mean_.fill(0);
	covariance_.setIdentity();
	sigma_points_.fill(0);
}

void Entity::PrintNIS()
{
	double sum = 0;
	double min = 9999;
	double max = 0;
	auto count = 0;

	for (auto i = NIS_.begin(); i != NIS_.end(); ++i)
	{
		if (*i < min) min = *i;
		if (*i > max) max = *i;
		sum += *i;
		count++;
	}

	auto mean = sum / count;
	std::cout << std::endl << "Mean NIS: " << mean;
	std::cout << std::endl << "Max NIS: " << max;
	std::cout << std::endl << "Min NIS: " << min << std::endl << std::endl;
}
