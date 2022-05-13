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
			geo::Vector tangent = (rv - rv.Dot(c.points.normal) * c.points.normal).Normalized();
			if(velocityAlongNormal > 0)
				continue;
			float e = std::min(a->restitution, b->restitution);
			geo::Vector rA = c.points.b - (a->GetCollider().GetCenter() + a->position);
			geo::Vector rB = c.points.a - (b->GetCollider().GetCenter() + b->position);
			f64 j = -(1 + e) * velocityAlongNormal * c.points.depth;
			j /= (a->GetInvMass() + b->GetInvMass() + SQRD(rA.Cross(c.points.normal)) * a->GetInvInertia() +
				SQRD(rB.Cross(c.points.normal)) * b->GetInvInertia());
			geo::Vector imp = j * c.points.normal;
			if (std::isnan(j) || j == std::numeric_limits<f64>::infinity())
				j = 0;
			if (!a->isStatic)
				a->ApplyImpulse(-imp * dt);
			if (!b->isStatic)
				b->ApplyImpulse(imp * dt);
			// Friction & rotation
			rv = b->velocity - a->velocity;
			tangent = (rv - rv.Dot(c.points.normal) * c.points.normal).Normalized();
			//f64 jt = -rv.Dot(tangent) / (a);
		}
	}
}