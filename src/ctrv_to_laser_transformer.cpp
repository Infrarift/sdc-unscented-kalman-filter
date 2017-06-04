#include "ctrv_to_laser_transformer.h"

CTRVToLaserTransformer::CTRVToLaserTransformer()
{
}

void CTRVToLaserTransformer::Convert(Entity& origin, Entity& target)
{
	for (auto i = 0; i < target.n_sigma_; i++) {
		target.sigma_points_(0, i) = origin.sigma_points_(0, i);
		target.sigma_points_(1, i) = origin.sigma_points_(1, i);
	}
}
