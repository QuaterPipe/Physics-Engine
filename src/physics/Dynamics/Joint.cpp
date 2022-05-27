#include "../../include/physics/Dynamics/Joint.hpp"

namespace physics
{
	DistanceJoint::DistanceJoint(Dynamicbody* a, Dynamicbody* b, f64 length) noexcept
	: length(length)
	{
		this->a = a;
		this->b = b;
	}

	void DistanceJoint::Update(f64 dt) noexcept
	{
		if (geo::DistanceSquared(a->position, b->position) < SQRD(length))
		{
	 		f64 dis = geo::Distance(a->position, b->position) - length;
			a->position += dis * (a->GetMass() / (a->GetMass() + b->GetMass())) * (b->position - a->position).Normalized();
			b->position += dis * (b->GetMass() / (a->GetMass() + b->GetMass())) * (a->position - b->position).Normalized();
		}
	}
}