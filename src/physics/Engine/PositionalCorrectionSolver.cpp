#include "physics/Engine/DynamicsWorld.hpp"

namespace physics
{
	void PositionalCorrectionSolver::Solve(std::vector<CollisionManifold>& collisions, f64 dt) noexcept
	{
		for (CollisionManifold& c : collisions)
		{
			if (!c.a->IsDynamic() || !c.b->IsDynamic()) continue;
			Dynamicbody* a = dynamic_cast<Dynamicbody*>(c.a);
			Dynamicbody* b = dynamic_cast<Dynamicbody*>(c.b);
			if (!a || !b) return;
			const f64 k_slop = 0.05; // Penetration allowance
			const f64 percent = 0.4; // Penetration percentage to correct
			geo::Vector2 correction = (std::max(c.points.depth - k_slop, 0.0) / (a->GetInvMass() * a->MassScaler() + b->GetInvMass() * b->MassScaler())) * c.points.normal * percent;
			if (!a->isStatic)
				a->Translate(-correction * a->GetInvMass() * a->MassScaler(), c.points.points, c.points.pointCount);
			if (!b->isStatic)
				b->Translate(correction * b->GetInvMass() * b->MassScaler(), c.points.points, c.points.pointCount);
		}
	}
}