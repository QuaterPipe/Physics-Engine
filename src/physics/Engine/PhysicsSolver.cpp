#include "../../include/physics/Engine/DynamicsWorld.hpp"
#include <iostream>
namespace physics
{
	void PositionalCorrectionSolve(Collision& c, f64 dt)
	{
		if (!c.a->IsDynamic() || !c.b->IsDynamic()) return;
		Rigidbody* a = (Rigidbody*) c.a;
		Rigidbody* b = (Rigidbody*) c.b;
		double percentage = 0.01;
		geo::Vector2 correction = c.points.depth / (a->GetMass() + b->GetMass()) * percentage * c.points.normal;
		//geo::Vector correction = std::max(c.points.depth - slop, 0.0) / (a->GetInvMass() + b->GetInvMass()) * percentage * c.points.normal;
		geo::Vector2 aPos = a->position;
		geo::Vector2 bPos = b->position;
		aPos -= a->GetInvMass() * correction;
		bPos += b->GetInvMass() * correction;
		
		if (a->isKinematic && !a->isStatic)
			a->position = aPos;
		if (b->isKinematic && !a->isStatic)
			b->position = bPos;
	}

	//credit to: https://gamedevelopment.tutsplus.com/tutorials/how-to-create-a-custom-2d-physics-engine-friction-scene-and-jump-table--gamedev-7756
	//and https://gamedevelopment.tutsplus.com/tutorials/how-to-create-a-custom-2d-physics-engine-the-basics-and-impulse-resolution--gamedev-6331
	void PhysicsSolver::Solve(std::vector<Collision>& collisions, f64 dt) noexcept
	{
		f64 maxJ = -std::numeric_limits<f64>::infinity();
		for (Collision& c: collisions)
		{
			if (!c.a->IsDynamic() || !c.b->IsDynamic()) continue;
			Dynamicbody* a = dynamic_cast<Dynamicbody*>(c.a);
			Dynamicbody* b = dynamic_cast<Dynamicbody*>(c.b);
			if (!a || !b) return;
			for (int i = 0; i < 50; i++)
			{
				maxJ = -std::numeric_limits<f64>::infinity();
				// Impulses
				// Calculate relative velocity in terms of the normal direction
				geo::Vector2 rv = b->velocity - a->velocity;
				f64 velocityAlongNormal = rv.Dot(c.points.normal);
				if(velocityAlongNormal > 0)
					return;
				geo::Vector2 tangent = (rv - rv.Dot(c.points.normal) * c.points.normal).Normalized();
				f64 e = std::min(a->restitution, b->restitution);
				//geo::Vector rA = c.points.b - (a->GetCollider().GetCenter() + a->position);
				//geo::Vector rB = c.points.a - (b->GetCollider().GetCenter() + b->position);
				f64 j = -(1 + e) * velocityAlongNormal * c.points.depth;
				j /= a->GetInvMass() + b->GetInvMass();
				geo::Vector2 imp = j * c.points.normal;
				f64 aPrcnt = a->GetMass() / (a->GetMass() + b->GetMass());
				f64 bPrcnt = b->GetMass() / (a->GetMass() + b->GetMass());
				if (std::isnan(j) || j == std::numeric_limits<f64>::infinity())
					j = 0;
				if (j > maxJ)
					maxJ = j;
				if (!a->isStatic)
					a->ApplyImpulse(dt, -imp * aPrcnt, c.points.a);
				if (!b->isStatic)
					b->ApplyImpulse(dt, imp * bPrcnt, c.points.a);
				// friction
				rv = b->velocity - a->velocity;
				tangent = rv - rv.Dot(c.points.normal) * c.points.normal;
				tangent.Normalize();
				e = std::min(a->restitution, b->restitution);
				j = -(1 + e) * rv.Dot(c.points.normal) * c.points.depth;
				j /= a->GetInvMass() + b->GetInvMass();
				f64 jt = -rv.Dot(tangent);
				jt /= a->GetMass() + b->GetMass();
				f64 mu = sqrt(SQRD(a->staticFriction) + SQRD(b->staticFriction));
				geo::Vector2 frictionImpulse;
				if (fabs(jt) < j * mu)
					frictionImpulse = jt * tangent;
				else
				{
					f64 kineticFriction = sqrt(SQRD(a->kineticFriction) + SQRD(b->kineticFriction));
					frictionImpulse = -j * tangent * kineticFriction;
				}
				if (!a->isStatic)
					a->ApplyImpulse(dt, -frictionImpulse);
				if (!b->isStatic)
					b->ApplyImpulse(dt, frictionImpulse);
				if (maxJ < EPSILON)
					break;
			}
		}
	}
}