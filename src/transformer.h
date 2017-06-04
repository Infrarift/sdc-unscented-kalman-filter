#pragma once
#include "entity.h"

class Transformer
{
public:
	virtual ~Transformer() = default;
	virtual void Transformer::Convert(Entity& origin, Entity& target) = 0;
};