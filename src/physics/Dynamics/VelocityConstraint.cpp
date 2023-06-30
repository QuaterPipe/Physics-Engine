#include "physics/Dynamics/Dynamicbody.hpp"

namespace physics
{	
	
	Constraint::Output VelocityConstraint::Calculate(f64 dt)
	{
		Output out;
		const geo::Vector2 rA = contactA - (a->GetCollider().GetCenter() + a->position);
		const geo::Vector2 rB = contactB - (b->GetCollider().GetCenter() + b->position);
		out.K = geo::Matrix(1, 1);
		out.K[0][0] = a->GetInvMass() + b->GetInvMass() + a->GetInvMass() * SQRD(rA.Cross(normal))
			+ b->GetInvMass() * SQRD(rB.Cross(normal));
		out.J = geo::Matrix(1, 6);
		out.J[0][0] = normal.x;
		out.J[0][1] = normal.y;
		out.J[0][2] = rA.Cross(normal);
		out.J[0][3] = -normal.x;
		out.J[0][4] = -normal.y;
		out.J[0][5] = rB.Cross(normal);
		const  geo::Vector2 relativeVel = (b->velocity + geo::Vector2::Cross(b->angularVelocity, rB) -
			a->velocity );
		const f64 restitutionBias = restitution * relativeVel.Dot(normal);
		const f64 errorBias = (penetrationBias / dt) * penetration;
		const geo::Vector2 result = normal * (errorBias + restitutionBias);
		out.B.SetSize(2);
		out.B[0] = result.x;
		out.B[1] = result.y;
		// -(J * (v + M^-1 * Fext * dt) + b)
		geo::Vector v(6), forceOverMass(6);
		v[0] = a->velocity.x;
		v[1] = a->velocity.y;
		v[2] = a->angularVelocity;
		v[3] = b->velocity.x;
		v[4] = b->velocity.y;
		v[5] = b->angularVelocity;
		forceOverMass[0] = a->force.x * a->GetInvMass() * dt;
		forceOverMass[1] = a->force.y * a->GetInvMass() * dt;
		forceOverMass[2] = a->angularForce * a->GetInvInertia() * dt;
		forceOverMass[3] = b->force.x * b->GetInvMass() * dt;
		forceOverMass[4] = b->force.y * b->GetInvMass() * dt;
		forceOverMass[5] = b->angularForce * b->GetInvInertia() * dt;
		out.rhs = -(out.J * (v + forceOverMass) + out.B);
		out.low.SetSize(6);
		out.high.SetSize(6);
		for (u32 i = 0; i < 6; i++)
		{
			out.low[i] = -std::numeric_limits<f64>::infinity();
			out.high[i] = std::numeric_limits<f64>::infinity();
		}
		return out;
	}
}