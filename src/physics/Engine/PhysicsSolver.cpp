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
			std::cerr<<"\n\n\n\n AHHHHHHHHHHHHHHHHH\n\n";
			if (!c.a->IsDynamic() || !c.b->IsDynamic()) continue;
			Dynamicbody* a = dynamic_cast<Dynamicbody*>(c.a);
			Dynamicbody* b = dynamic_cast<Dynamicbody*>(c.b);
			if (!a || !b) continue;
			// Calculate relative velocity in terms of the normal direction
			geometry::Vector rv = b->velocity - a->velocity;
			float velocityAlongNormal = rv.Dot(c.points.normal);
			// Do not resolve if velocities are separating
			if(velocityAlongNormal > 0)
				continue;
			// Calculate restitution
			float e = std::min(a->restitution, b->restitution);
			// Calculate impulse scalar
			float j = -(1 + e) * velocityAlongNormal;
			j /= 1 / a->GetMass() + 1 / b->GetMass();
			geometry::Vector impulse = j * c.points.normal;
			if (!a->isStatic)
				a->ApplyForce(b->GetMass() / (a->GetMass() + b->GetMass()) * -impulse, c.points.a);
			if (!b->isStatic)
				b->ApplyForce(a->GetMass() / (a->GetMass() + b->GetMass()) * impulse, c.points.b);
		}
	}
}