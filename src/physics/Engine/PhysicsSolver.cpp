#include "../../include/physics/Engine/World.hpp"
#include "../../include/physics/Tools/OstreamOverloads.hpp"
#include <iostream>

namespace physics
{
	//credit to: https://gamedevelopment.tutsplus.com/tutorials/how-to-create-a-custom-2d-physics-engine-friction-scene-and-jump-table--gamedev-7756
	//and https://gamedevelopment.tutsplus.com/tutorials/how-to-create-a-custom-2d-physics-engine-the-basics-and-impulse-resolution--gamedev-6331
	void PhysicsSolver::Solve(std::vector<Collision>& collisions, f64 dt) noexcept
	{
		for (Collision& c: collisions)
		{
			if (!c.a->IsDynamic() || !c.b->IsDynamic()) continue;
			Dynamicbody* a = dynamic_cast<Dynamicbody*>(c.a);
			Dynamicbody* b = dynamic_cast<Dynamicbody*>(c.b);
			if (!a || !b) continue;
			// Impulses
			// Calculate relative velocity in terms of the normal direction
			geo::Vector rv = b->velocity - a->velocity;
			f64 velocityAlongNormal = rv.Dot(c.points.normal);
			if(velocityAlongNormal > 0)
				continue;
			geo::Vector tangent = (rv - rv.Dot(c.points.normal) * c.points.normal).Normalized();
			f64 e = std::min(a->restitution, b->restitution);
			geo::Vector rA = c.points.b - (a->GetCollider().GetCenter() + a->position);
			geo::Vector rB = c.points.a - (b->GetCollider().GetCenter() + b->position);
			f64 j = -(1 + e) * velocityAlongNormal * c.points.depth;
			j /= a->GetInvMass() + b->GetInvMass();
			geo::Vector imp = j * c.points.normal;
			f64 aPrcnt = a->GetMass() / (a->GetMass() + b->GetMass());
			f64 bPrcnt = b->GetMass() / (a->GetMass() + b->GetMass());
			if (std::isnan(j) || j == std::numeric_limits<f64>::infinity())
				j = 0;
			if (!a->isStatic)
				a->ApplyImpulse(-imp * aPrcnt, c.points.a);
			if (!b->isStatic)
				b->ApplyImpulse(imp * bPrcnt, c.points.a);
			// Friction & rotation
			rv = b->velocity - a->velocity;
			tangent = (rv - rv.Dot(c.points.normal) * c.points.normal).Normalized();
			//f64 jt = -rv.Dot(tangent) / (a);
		}
	}
}