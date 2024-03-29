#include "physics/Engine/DynamicsWorld.hpp"
#include <iostream>
namespace physics
{
	geo::Vector ProjectedGaussianEliminationSolve(geo::Matrix matrix, geo::Vector right, f64 relaxation,
		i32 iterations, geo::Vector lo, geo::Vector hi)
	{
		// Validation omitted
		geo::Vector x = right;
		double delta;
		// Gauss-Seidel with Successive OverRelaxation Solver
		for (int k = 0; k < iterations; ++k)
		{
			for (unsigned int i = 0; i < right.GetSize(); ++i)
			{
				delta = 0.;
				for (int j = 0; j < i; ++j)
					delta += matrix(i, j) * x[j];
				for (unsigned int j = i + 1; j < right.GetSize(); ++j)
					delta += matrix(i, j) * x[j];
				delta = (right[i] - delta) / matrix(i, i);
				x[i] += relaxation * (delta - x[i]);
				// Project the solution within the lower and higher limits
				if (x[i] < lo[i])
					x[i] = lo[i];
				if (x[i] > hi[i])
					x[i] = hi[i];
			}
		}
		return x;
	}
	
	void PhysicsSolver::Solve(std::vector<CollisionManifold>& collisions, f64 dt) noexcept
	{
		for (CollisionManifold& c: collisions)
		{
			if (!c.a->IsDynamic() || !c.b->IsDynamic()) continue;
			Dynamicbody* a = dynamic_cast<Dynamicbody*>(c.a);
			Dynamicbody* b = dynamic_cast<Dynamicbody*>(c.b);
			if (!a || !b) return;
			f64 e = geo::Min(a->restitution, b->restitution);
			f64 sf = sqrt(SQRD(a->staticFriction) + SQRD(b->staticFriction));
			f64 kf = sqrt(SQRD(a->kineticFriction) + SQRD(b->kineticFriction));
			for (int i = 0; i < c.points.pointCount; i++)
			{
				geo::Vector2 ra = c.points.points[i] - (a->transform.GetPosition() + a->transform.GetCOM());
				geo::Vector2 rb = c.points.points[i] - (b->transform.GetPosition() + b->transform.GetCOM());
				geo::Vector2 rv = b->velocity + geo::Vector2::Cross(b->angularVelocity, rb) -
					a->velocity - geo::Vector2::Cross(a->angularVelocity, ra);
				if (rv.GetMagnitudeExact() < (dt * gravity).GetMagnitudeExact() + EPSILON)
					e = 0.0;
			}
			for (int i = 0; i < c.points.pointCount; i++)
			{
				geo::Vector2 ra = c.points.points[i] - (a->transform.GetPosition() + a->transform.GetCOM());
				geo::Vector2 rb = c.points.points[i] - (b->transform.GetPosition() + b->transform.GetCOM());
				geo::Vector2 rv = b->velocity + geo::Vector2::Cross(b->angularVelocity, rb) -
					a->velocity - geo::Vector2::Cross(a->angularVelocity, ra);
				f64 contactVel = rv.Dot(c.points.normal);
				if (contactVel > 0)
					break;
				f64 raCrossN = ra.Cross(c.points.normal);
				f64 rbCrossN = rb.Cross(c.points.normal);
				f64 invMassSum = a->GetInvMass() + b->GetInvMass() + SQRD(raCrossN) * a->GetInvInertia() +
					SQRD(rbCrossN) * b->GetInvInertia();
				f64 j = -(1.0 + e) * contactVel;
				j /= invMassSum;
				//j /= (f64)c.points.points.size();
				geo::Vector2 impulse = c.points.normal * j;
				a->ApplyImpulse(-impulse, ra);
				b->ApplyImpulse(impulse, rb);

				rv = b->velocity + geo::Vector2::Cross(b->angularVelocity, rb) -
					a->velocity - geo::Vector2::Cross(a->angularVelocity, ra);

				geo::Vector2 t = rv - (c.points.normal * rv.Dot(c.points.normal));
				t.Normalize();

				f64 jt = -rv.Dot(t);
				jt /= invMassSum;
				// jt /= (f64)c.points.points.size();

				if (geo::Equal(jt, 0.0))
					continue;
				geo::Vector2 tangentImpulse;
				if (std::abs(jt) < j * sf)
					tangentImpulse = t * jt;
				else
					tangentImpulse = t * -j * kf;
				a->ApplyImpulse(-tangentImpulse, ra);
				b->ApplyImpulse(tangentImpulse, rb);
			}
		}	
	}
}