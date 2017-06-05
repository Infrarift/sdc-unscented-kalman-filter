#pragma once
#include "entity.h"

class Transformer
{
public:
	virtual ~Transformer() = default;
	virtual void Convert(Entity& origin, Entity& target) = 0;
};