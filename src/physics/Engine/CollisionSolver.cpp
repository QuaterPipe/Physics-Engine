#include "physics/Engine/DynamicsWorld.hpp"
#include <iostream>
namespace physics
{
	void CollisionSolver::Solve(std::vector<CollisionManifold>& collisions, f64 dt) noexcept
	{
		for (CollisionManifold& c: collisions)
		{
			if (!c.a->IsDynamic() || !c.b->IsDynamic())
				continue;
			Dynamicbody* a = dynamic_cast<Dynamicbody*>(c.a);
			Dynamicbody* b = dynamic_cast<Dynamicbody*>(c.b);
			if (!a || !b)
				return;
			f64 e = Min(a->restitution, b->restitution);
			f64 sf = sqrt(SQRD(a->staticFriction) + SQRD(b->staticFriction));
			f64 kf = sqrt(SQRD(a->kineticFriction) + SQRD(b->kineticFriction));
			for (int i = 0; i < c.points.pointCount; i++)
			{
				Vector2 ra = c.points.points[i] - (a->transform.GetPosition() + a->transform.GetCOM());
				Vector2 rb = c.points.points[i] - (b->transform.GetPosition() + b->transform.GetCOM());
				Vector2 rv = b->velocity + Vector2::Cross(b->angularVelocity, rb) -
					a->velocity - Vector2::Cross(a->angularVelocity, ra);
				if (rv.GetMagnitudeExact() < (dt * gravity).GetMagnitudeExact() + EPSILON)
					e = 0.0;
			}
			for (int i = 0; i < c.points.pointCount; i++)
			{
				Vector2 ra = c.points.points[i] - (a->transform.GetPosition() + a->transform.GetCOM());
				Vector2 rb = c.points.points[i] - (b->transform.GetPosition() + b->transform.GetCOM());
				Vector2 rv = b->velocity + Vector2::Cross(b->angularVelocity, rb) -
					a->velocity - Vector2::Cross(a->angularVelocity, ra);
				f64 contactVel = rv.Dot(c.points.normal);
				if (contactVel > 0)
					break;
				f64 raCrossN = ra.Cross(c.points.normal);
				f64 rbCrossN = rb.Cross(c.points.normal);
				f64 invMassSum = a->GetInvMass() * a->MassScaler() + b->GetInvMass() * b->MassScaler() + SQRD(raCrossN) * a->GetInvInertia() +
					SQRD(rbCrossN) * b->GetInvInertia();
				f64 j = -(1.0 + e) * contactVel;
				j /= invMassSum;
				j /= (f64)c.points.pointCount;
				Vector2 impulse = c.points.normal * j;
				a->ApplyImpulse(-impulse, ra);
				b->ApplyImpulse(impulse, rb);

				rv = b->velocity + Vector2::Cross(b->angularVelocity, rb) -
					a->velocity - Vector2::Cross(a->angularVelocity, ra);

				Vector2 t = rv - (c.points.normal * rv.Dot(c.points.normal));
				t.Normalize();

				f64 jt = -rv.Dot(t);
				jt /= invMassSum;
				jt /= (f64)c.points.pointCount;

				if (Equal(jt, 0.0))
					continue;
				Vector2 tangentImpulse;
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