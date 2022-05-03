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
			geo::Vector tangent = rv - rv.Dot(c.points.normal) * c.points.normal;
			f64 velocityAlongNormal = rv.Dot(c.points.normal);
			// Do not resolve if velocities are separating
			if(velocityAlongNormal > 0)
				continue;
			// Calculate restitution
			float e = std::min(a->restitution, b->restitution);
			// Calculate impulse scalar
			f64 j = -(1 + e) * velocityAlongNormal;
			geo::Vector rA = c.points.b - (a->GetCollider().GetCenter() + a->position);
			geo::Vector rB = c.points.a - (b->GetCollider().GetCenter() + b->position);
			geo::Vector imp =  (-(1 + e) * (rv * c.points.normal)) /
				(a->GetInvMass() + b->GetInvMass() + (SQRD(rA.Cross(tangent)) * a->GetInvInertia())
				+ (SQRD(rB.Cross(tangent)) * b->GetInvInertia()));
			std::cout<<imp<<"\n";
			std::cout<<"a vel: "<<a->velocity<<" b vel: "<<b->velocity<<"\n";
			j /= a->GetInvMass() + b->GetInvMass();
			if (std::isnan(j) || j == std::numeric_limits<f64>::infinity())
				j = 0;
			if (!a->isStatic)
				a->ApplyImpulse(-imp, c.points.a);
			if (!b->isStatic)
				b->ApplyImpulse(imp, c.points.b);
			//applying friction
			rv = b->velocity - a->velocity;
			tangent = rv - rv.Dot(c.points.normal) * c.points.normal;
			tangent.Normalize();
			f64 jt = -rv.Dot(tangent);
			jt /= a->GetInvMass() + b->GetInvMass();
			f64 mu = sqrt(SQRD(a->staticFriction) + SQRD(b->staticFriction));
			geo::Vector frictionImpulse;
			if (fabs(jt) < j * mu)
				frictionImpulse = jt * tangent;
			else
			{
				f64 dynFric = sqrt(SQRD(a->kineticFriction) + SQRD(b->kineticFriction));
				frictionImpulse = -j * tangent * dynFric;
			}
			/*if (!a->isStatic)
				a->ApplyForce(-frictionImpulse * dt, c.points.a);
			if (!b->isStatic)
				b->ApplyForce(frictionImpulse * dt, c.points.b);*/
		}
	}
}