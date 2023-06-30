#pragma once
#include "Constraint.hpp"

namespace physics
{
	struct VelocityConstraint : public Constraint
	{
		geo::Vector2 normal;
		geo::Vector2 contactA;
		geo::Vector2 contactB;
		f64 penetrationBias = 0.2;
		f64 restitution;
		f64 penetration;
		virtual Output Calculate(f64 dt) override;
	
	};
}