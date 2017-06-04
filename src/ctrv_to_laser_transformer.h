#pragma once
#include "transformer.h"

class CTRVToLaserTransformer : public Transformer
{
public:
	CTRVToLaserTransformer();
	void CTRVToLaserTransformer::Convert(Entity& origin, Entity& target) override;
};
