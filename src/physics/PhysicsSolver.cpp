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
			// Calculate relative velocity in terms of the normal direction
			geometry::Vector rv = b->GetVelocity() - a->GetVelocity();
			float velocityAlongNormal = rv.Dot(c.points.normal);
			// Do not resolve if velocities are separating
			if(velocityAlongNormal > 0)
				continue;
			// Calculate restitution
			float e = std::min(a->GetRestitution(), b->GetRestitution());
			// Calculate impulse scalar
			float j = -(1 + e) * velocityAlongNormal;
			j /= 1 / a->GetMass() + 1 / b->GetMass();
			geometry::Vector impulse = j * c.points.normal;
			std::cerr<<velocityAlongNormal<<"\n";
			std::cerr<<a->GetVelocity()<<"\n";
			std::cerr<<b->GetVelocity()<<"\n";
			std::cerr<<"rv: "<<rv<<"\n";
			std::cerr<<"normal: "<<c.points.normal<<"\n";
			std::cerr<<"a force: "<<b->GetMass() / (a->GetMass() + b->GetMass()) * -impulse<<"\n";
			std::cerr<<"b force: "<<a->GetMass() / (a->GetMass() + b->GetMass()) * impulse<<"\n";
			a->ApplyForce(b->GetMass() / (a->GetMass() + b->GetMass()) * -impulse, c.points.a);
			b->ApplyForce(a->GetMass() / (a->GetMass() + b->GetMass()) * impulse, c.points.b);
			
		}
	}
}