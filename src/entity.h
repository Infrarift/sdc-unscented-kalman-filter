#pragma once
#include "Eigen/Geometry"
#include <vector>

class Entity
{
public:
	Entity();
	explicit Entity(int dimensions, int n_sigma);
	explicit Entity(int dimensions, int n_sigma, double noise[]);
	~Entity();

	Eigen::MatrixXd covariance_;
	Eigen::MatrixXd sigma_points_;
	Eigen::VectorXd mean_;
	std::vector<double> NIS_;

	int dimensions_;
	int n_sigma_;

	void NormalizeVector(Eigen::VectorXd& vector);
	void NormalizeAngles();
	void Reset();

	void Entity::SetNoise(double noise[]);
	void Entity::AddNormalizationIndex(int index);
	Eigen::MatrixXd Entity::GetNoiseMatrix();
	Eigen::VectorXd Entity::GetNoiseVector();
	void Entity::PrintNIS();

private:
	void Entity::Initialize(int dimensions, int n_sigma);
	Eigen::VectorXd noise_vector_;
	Eigen::MatrixXd noise_matrix_;
	std::vector<int> normalize_index_;
};

