#include "Constraint.hpp"

namespace physics
{
	struct VelocityConstraint : public Constraint
	{
		geo::Vector2 normal;
		geo::Vector2 contactA;
		geo::Vector2 contactB;
		f64 penetrationBias = 0.2;
		f64 restitution;
		f64 penetration;
		virtual Output Calculate(f64 dt) override
		{
			Output o;
			const geo::Vector2 rA = contactA - (a->GetCollider().GetCenter() + a->position);
			const geo::Vector2 rB = contactB - (b->GetCollider().GetCenter() + b->position);
			o.K.SetHeight(1);
			o.K.SetWidth(1);
			o.K[0][0] = a->GetInvMass() + b->GetInvMass() + a->GetInvMass() * SQRD(rA.Cross(normal))
				+ b->GetInvMass() * SQRD(rB.Cross(normal));
			o.J.SetHeight(6);
			o.J.SetWidth(1);
			o.J[0][0] = normal.x;
			o.J[0][1] = normal.y;
			o.J[0][2] = rA.Cross(normal);
			o.J[0][3] = -normal.x;
			o.J[0][4] = -normal.y;
			o.J[0][5] = rB.Cross(normal);
			const  geo::Vector2 relativeVel = (b->velocity + geo::Vector2::Cross(b->angularVelocity, rB) -
				a->velocity );
			const f64 restitutionBias = restitution * relativeVel.Dot(normal);
			const f64 errorBias = (penetrationBias / dt) * penetration;
			const geo::Vector2 result = normal * (errorBias + restitutionBias);
			o.B.SetSize(2);
			o.B[0] = result.x;
			o.B[1] = result.y;
			o.Limits.SetHeight(6);
			o.Limits.SetWidth(2);
			for (u32 i = 0; i < 6; i++)
				o.Limits[i][0] = -std::numeric_limits<f64>::infinity();
			for (u32 i = 0; i < 6; i++)
				o.Limits[i][1] = std::numeric_limits<f64>::infinity();
			return o;
		}
	
	};
}