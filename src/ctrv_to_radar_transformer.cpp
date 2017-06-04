#include "ctrv_to_radar_transformer.h"

CTRVToRadarTransformer::CTRVToRadarTransformer()
{
}

void CTRVToRadarTransformer::Convert(Entity& origin, Entity& target)
{
	for (auto i = 0; i < target.n_sigma_; i++)
	{
		auto p_x = origin.sigma_points_(0, i);
		auto p_y = origin.sigma_points_(1, i);
		auto v = origin.sigma_points_(2, i);
		auto yaw = origin.sigma_points_(3, i);

		auto v1 = cos(yaw) * v;
		auto v2 = sin(yaw) * v;

		auto px_py_sq = sqrt(p_x * p_x + p_y * p_y);
		target.sigma_points_(0, i) = px_py_sq;
		target.sigma_points_(1, i) = atan2(p_y, p_x);
		target.sigma_points_(2, i) = px_py_sq == 0 ? 0 : (p_x * v1 + p_y * v2) / px_py_sq;
	}
}
