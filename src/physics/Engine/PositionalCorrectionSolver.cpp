#include "physics/Engine/DynamicsWorld.hpp"

namespace physics
{
	void PositionalCorrectionSolver::Solve(std::vector<Collision>& collisions, f64 dt) noexcept
	{
		for (Collision& c : collisions)
		{
			if (!c.a->IsDynamic() || !c.b->IsDynamic()) continue;
			Dynamicbody* a = dynamic_cast<Dynamicbody*>(c.a);
			Dynamicbody* b = dynamic_cast<Dynamicbody*>(c.b);
			if (!a || !b) return;
			const f64 k_slop = 0.05; // Penetration allowance
			const f64 percent = 0.2; // Penetration percentage to correct
			geo::Vector2 correction = (std::max(c.points.depth - k_slop, 0.0) / (a->GetInvMass() + b->GetInvMass())) * c.points.normal * percent;
			if (!a->isStatic)
				a->transform.Translate(-correction * a->GetInvMass());
			if (!b->isStatic)
				b->transform.Translate(correction * b->GetInvMass());
		}
	}
}