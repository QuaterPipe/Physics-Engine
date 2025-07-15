#include "physics/Dynamics/Constraint.hpp"
#include "physics/Dynamics/Dynamicbody.hpp"

namespace physics
{
	AnchorConstraint::AnchorConstraint(Dynamicbody* a, geo::Vector2 anchorPoint, geo::Vector2 pointOnBody) noexcept
		: anchorPoint(anchorPoint), pointOnBody(pointOnBody)
	{
		this->a = a;
	}

	void AnchorConstraint::UpdateConstraint(f64 dt) noexcept
	{
	}

	//AngleConstraint(Dynamicbody* a, f64 angle)
}