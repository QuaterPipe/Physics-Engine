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
			// Calculate relative velocity in terms of the normal direction
			geometry::Vector rv = b->velocity - a->velocity;
			geometry::Vector tangent = rv - rv.Dot(c.points.normal) * c.points.normal;
			tangent.Normalize();
			f64 velocityAlongNormal = rv.Dot(c.points.normal);
			// Do not resolve if velocities are separating
			if(velocityAlongNormal > 0)
				continue;
			// Calculate restitution
			float e = std::min(a->restitution, b->restitution);
			// Calculate impulse scalar
			float j = -(1 + e) * velocityAlongNormal;
			j /= a->GetInvMass() + b->GetInvMass();
			j /= a->GetInvMass() + b->GetInvMass();
			f64 jt = -rv.Dot(tangent);
			jt /= (a->GetInvMass() + b->GetInvMass());
			f64 mu = sqrt(SQRD(a->staticFriction) + SQRD(b->staticFriction));
			geometry::Vector frictionImpulse;
			if (fabs(jt) < j * mu)
				frictionImpulse = jt * tangent;
			else
			{
				f64 dynFric = sqrt(SQRD(a->kineticFriction) + SQRD(b->kineticFriction));
				frictionImpulse = -j * tangent * dynFric;
			}
			geometry::Vector impulse = j * c.points.normal;
			//ratio
			if (!a->isStatic)
			{
				a->ApplyForce(-impulse * a->GetInvMass() * (a->GetMass() / (a->GetMass() + b->GetMass())));
				a->ApplyForce(-frictionImpulse * a->GetInvMass());
			}
			if (!b->isStatic)
			{
				b->ApplyForce(impulse * b->GetInvMass() * (b->GetMass() / (a->GetMass() + b->GetMass())));
				b->ApplyForce(frictionImpulse * b->GetInvMass());
			}
		}
	}
}