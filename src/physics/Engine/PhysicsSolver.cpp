#include "../../include/physics/Engine/World.hpp"
#include "../../include/physics/Tools/OstreamOverloads.hpp"
#include <iostream>
#include <cmath>
namespace physics
{
	void ImpulseSolve(Collision& c, f64 dt)
	{
		Dynamicbody* a = dynamic_cast<Dynamicbody*>(c.a);
		Dynamicbody* b = dynamic_cast<Dynamicbody*>(c.b);
		if (!a || !b) return;
		// Impulses
		// Calculate relative velocity in terms of the normal direction
		geo::Vector rv = b->velocity - a->velocity;
		f64 velocityAlongNormal = rv.Dot(c.points.normal);
		if(velocityAlongNormal > 0)
			return;
		geo::Vector tangent = (rv - rv.Dot(c.points.normal) * c.points.normal).Normalized();
		f64 e = std::min(a->restitution, b->restitution);
		//geo::Vector rA = c.points.b - (a->GetCollider().GetCenter() + a->position);
		//geo::Vector rB = c.points.a - (b->GetCollider().GetCenter() + b->position);
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

	static void PositionalCorrectionSolve(Collision& c, f64 dt)
	{
		if (!c.a->IsDynamic() || !c.b->IsDynamic()) return;
		Rigidbody* a = (Rigidbody*) c.a;
		Rigidbody* b = (Rigidbody*) c.b;
		double percentage = -100000000;
		geo::Vector correction = c.points.depth / (a->GetMass() + b->GetMass()) * percentage * c.points.normal;
		//geo::Vector correction = std::max(c.points.depth - slop, 0.0) / (a->GetInvMass() + b->GetInvMass()) * percentage * c.points.normal;
		geo::Vector aPos = a->position;
		geo::Vector bPos = b->position;
		aPos -= a->GetInvMass() * correction;
		bPos += b->GetInvMass() * correction;
		
		if (a->isKinematic && !a->isStatic)
			a->position = aPos;
		if (b->isKinematic && !a->isStatic)
			b->position = bPos;
	}

	static void FrictionSolve(Collision& c, f64 dt)
	{
		Dynamicbody* a = dynamic_cast<Dynamicbody*>(c.a);
		Dynamicbody* b = dynamic_cast<Dynamicbody*>(c.b);
		if (!a || !b) return;
		geo::Vector rv = b->velocity - a->velocity;
		geo::Vector tangent = rv - rv.Dot(c.points.normal) * c.points.normal;
		tangent.Normalize();
		f64 e = std::min(a->restitution, b->restitution);
		f64 j = -(1 + e) * rv.Dot(c.points.normal) * c.points.depth;
		j /= a->GetInvMass() + b->GetInvMass();
		f64 jt = -rv.Dot(tangent);
		jt /= a->GetMass() + b->GetMass();
		f64 mu = sqrt(SQRD(a->staticFriction) + SQRD(b->staticFriction));
		geo::Vector frictionImpulse;
		if (fabs(jt) < j * mu)
			frictionImpulse = jt * tangent;
		else
		{
			f64 kineticFriction = sqrt(SQRD(a->kineticFriction) + SQRD(b->kineticFriction));
			frictionImpulse = -j * tangent * kineticFriction;
		}
		if (!a->isStatic)
			a->ApplyImpulse(-frictionImpulse);
		if (!b->isStatic)
			b->ApplyImpulse(frictionImpulse);
	}
	//credit to: https://gamedevelopment.tutsplus.com/tutorials/how-to-create-a-custom-2d-physics-engine-friction-scene-and-jump-table--gamedev-7756
	//and https://gamedevelopment.tutsplus.com/tutorials/how-to-create-a-custom-2d-physics-engine-the-basics-and-impulse-resolution--gamedev-6331
	void PhysicsSolver::Solve(std::vector<Collision>& collisions, f64 dt) noexcept
	{
		for (Collision& c: collisions)
		{
			if (!c.a->IsDynamic() || !c.b->IsDynamic()) continue;
			ImpulseSolve(c, dt);
		}
		int i = 0;
		for (Collision& c: collisions)
		{
			if (!c.a->IsDynamic() || !c.b->IsDynamic()) continue;
			if (i == 2)
				std::cout<<c.a->position<<" "<<c.b->position<<"\n";
			//ImpulseSolve(c, dt);
			PositionalCorrectionSolve(c, dt);
			i++;
		}
	}
}