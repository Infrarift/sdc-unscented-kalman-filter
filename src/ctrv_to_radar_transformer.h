#pragma once
#include "transformer.h"

class CTRVToRadarTransformer : public Transformer
{
public:
	CTRVToRadarTransformer();
	void CTRVToRadarTransformer::Convert(Entity& origin, Entity& target) override;
};
