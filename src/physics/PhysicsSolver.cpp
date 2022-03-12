#include "../include/physics/World.hpp"
#include "../include/physics/OstreamOverloads.hpp"
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
			Rigidbody* a = (Rigidbody*) c.a;
			Rigidbody* b = (Rigidbody*) c.b;
			double velocityAlongNormal = (b->GetVelocity() - a->GetVelocity()).Dot(c.points.normal);
			// impulse scalar
			double ImpulseScalar = -(1 + std::min(a->GetRestitution(), b->GetRestitution())) *
				velocityAlongNormal;
			ImpulseScalar /= (a->GetInvMass() + b->GetInvMass());
			// Vr = b vel - a vel
			//tangent = Vr - (Vr dot Normal) * normal
			geometry::Vector tangent = (b->GetVelocity() - a->GetVelocity()) 
			- ((b->GetVelocity() - a->GetVelocity()).Dot(c.points.normal)) * c.points.normal;
			tangent.Normalize();
			// magnitude of friction
			double magnitude = (b->GetVelocity() - a->GetVelocity()).Dot(tangent);
			magnitude /= b->GetInvMass() + a->GetInvMass();
			// static coefficient
			double mu = sqrt(SQRD(a->GetStaticFriction()) + SQRD(b->GetStaticFriction()));
			// using coloumbs law to clamp the friction
			geometry::Vector frictionImpulse;
			if (fabs(magnitude) < ImpulseScalar * mu)
				frictionImpulse = magnitude * tangent;
			else
			{
				double dynaFriction = sqrt(SQRD(a->GetKineticFriction()) + SQRD(b->GetKineticFriction()));
				frictionImpulse = -ImpulseScalar * tangent * dynaFriction;
			}
			// applying impulse force
			std::cerr<<-(a->GetInvMass() * frictionImpulse)<<"\n";
			std::cerr<<c.points.a<<" "<<c.points.b<<"\n";
			a->ApplyForce(-(a->GetInvMass() * c.points.normal * ImpulseScalar), c.points.a);
			b->ApplyForce((b->GetInvMass() * c.points.normal * ImpulseScalar), c.points.b);
			// applying friction force
			a->ApplyForce(-(a->GetInvMass() * frictionImpulse), c.points.b); // using where b touched a for friction
			b->ApplyForce((b->GetInvMass() * frictionImpulse), c.points.a); // using where a touched b for friction
		}
	}
}