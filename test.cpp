#include <iostream>
using f64 = double;
struct State
{
	f64 x, v;
};
struct Derivative
{
	f64 dx, dv;
};

f64 Acceleration(const State & state, f64 t )
{
	const f64 k = 15.0;
	const f64 b = 0.1;
	return -k * state.x - b * state.v;
}
Derivative Eval(const State& initial, f64 t, f64 dt, const Derivative& d)
{
	State state;
	state.x = initial.x + d.dx*dt;
	state.v = initial.v + d.dv*dt;

	Derivative output;
	output.dx = state.v;
	output.dv = Acceleration( state, t+dt );
	return output;
}
void Integrate( State & state, 
				double t, 
				float dt )
{
	Derivative a,b,c,d;

	a = Eval( state, t, 0.0f, Derivative() );
	b = Eval( state, t, dt*0.5f, a );
	c = Eval( state, t, dt*0.5f, b );
	d = Eval( state, t, dt, c );

	f64 dxdt = 1.0f / 6.0f * 
		( a.dx + 2.0f * ( b.dx + c.dx ) + d.dx );
	
	f64 dvdt = 1.0f / 6.0f * 
		( a.dv + 2.0f * ( b.dv + c.dv ) + d.dv );

	state.x = state.x + dxdt * dt;
	state.v = state.v + dvdt * dt;
}

int main(int argc, char** args)
{
	State s;
	s.x = std::atof(args[1]);
	s.v = std::atof(args[2]);
	Integrate(s, 0, 1);
	std::cout<<s.x<<" "<<s.v<<"\n";
	return 0;
}
/*f64 e = std::min(a->restitution, b->restitution);
			for (int o = 0; o < 100; o++)
			{
				for (int i = 0; i < c.points.contactPoints.size(); i++)
				{
					geo::Vector2 rA = c.points.contactPoints[i] - (a->GetCollider().GetCenter() + a->position);
					geo::Vector2 rB = c.points.contactPoints[i] - (b->GetCollider().GetCenter() + b->position);
					geo::Vector2 rv = b->velocity + geo::Vector2::Cross(b->angularVelocity, rB) -
						a->velocity - geo::Vector2::Cross(a->angularVelocity, rA);
					f64 velocityAlongNormal = rv.Dot(c.points.normal);
					if(rv.GetMagnitudeSquared() < (dt * gravity).GetMagnitudeSquared() + EPSILON)
						e = 0.0;
				}
				for (int i = 0; i < c.points.contactPoints.size(); i++)
				{
					// Impulses
					// Calculate relative velocity in terms of the normal direction
					geo::Vector2 rA = c.points.contactPoints[i] - (a->GetCollider().GetCenter() + a->position);
					geo::Vector2 rB = c.points.contactPoints[i] - (b->GetCollider().GetCenter() + b->position);
					geo::Vector2 rv = b->velocity + geo::Vector2::Cross(b->angularVelocity, rB) -
						a->velocity - geo::Vector2::Cross(a->angularVelocity, rA);
					f64 velocityAlongNormal = rv.Dot(c.points.normal);
					if(velocityAlongNormal > 0)
						continue;
					f64 invMassSum = a->GetInvMass() + b->GetInvMass() + SQRD(rA.Cross(c.points.normal)) * a->GetInvInertia() +
						SQRD(rB.Cross(c.points.normal)) * b->GetInvInertia();
					f64 j = -(1.0 + e) * velocityAlongNormal;// * c.points.depth;
					j /= invMassSum;
					j /= c.points.contactPoints.size();
					geo::Vector2 imp = j * c.points.normal;
					if (std::isnan(j) || j == std::numeric_limits<f64>::infinity())
						j = 0;
					//j = j < 0 ? 0 : j;
					if (!a->isStatic)
						a->ApplyImpulse(dt, -imp * c.points.depth, rA);
					if (!b->isStatic)
						b->ApplyImpulse(dt, imp * c.points.depth, rB);
					// friction
					geo::Vector2 tangent = (rv - rv.Dot(c.points.normal) * c.points.normal).Normalized();
					rv = b->velocity + geo::Vector2::Cross(b->angularVelocity, rB) -
						a->velocity - geo::Vector2::Cross(a->angularVelocity, rA);
					tangent = rv - (c.points.normal * rv.Dot(c.points.normal));
					tangent.Normalize();
					f64 jt = -rv.Dot(tangent);
					jt /= invMassSum;
					jt /= c.points.contactPoints.size();
					if (jt < EPSILON)
						continue;

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
						a->ApplyImpulse(dt, -frictionImpulse, rA);
					if (!b->isStatic)
						b->ApplyImpulse(dt, frictionImpulse, rB);
				}
			}*/
/*
geo::Vector2 rA = c.points.contactPoints[0] - (a->GetCollider().GetCenter() + a->position);
			geo::Vector2 rB = c.points.contactPoints[0] - (b->GetCollider().GetCenter() + b->position); //shouldn't it be b.transform.TransformVector()?
			geo::Matrix J(6, 1);
			J[0][0] = c.points.normal.x;
			J[1][0] = c.points.normal.y;
			J[2][0] = rA.Cross(c.points.normal);
			J[3][0] = -c.points.normal.x;
			J[4][0] = -c.points.normal.y;
			J[5][0] = -rB.Cross(c.points.normal);
			geo::Vector V(6);
			V[0] = a->velocity.x;
			V[1] = a->velocity.y;
			V[2] = a->angularVelocity;
			V[3] = b->velocity.x;
			V[4] = b->velocity.y;
			V[5] = b->angularVelocity;
			geo::Matrix M(6, 6), MInv(6, 6);
			M[0][0] = a->GetMass();
			M[1][1] = a->GetMass();
			M[2][2] = a->GetInertia();
			M[3][3] = b->GetMass();
			M[4][4] = b->GetMass();
			M[5][5] = b->GetInertia();
			M.Inverse(MInv);
			geo::Matrix Jt(6, 1);
			for (i32 i = 0; i < 6; i++)
				Jt[i][0] = J[0][i];
			auto lambda = ProjectedGaussianEliminationSolve(J * MInv * Jt, -(J * V), )
*/

/*
geo::Vector ProjectedGaussianEliminationSolve (geo::Matrix matrix, geo::Vector right, f64 relaxation,
		i32 iterations, geo::Vector lo, geo::Vector hi)
	{
		// Validation omitted
		geo::Vector x = right;
		double delta;
		// Gauss-Seidel with Successive OverRelaxation Solver
		for (int k = 0; k < iterations; ++k)
		{
			for (int i = 0; i < right.GetSize(); ++i)
			{
				delta = 0.;
				for (int j = 0; j < i; ++j)
					delta += matrix[i][j] * x[j];
				for (int j = i + 1; j < right.GetSize(); ++j)
					delta += matrix[i][j] * x[j];
				delta = (right[i] - delta) / matrix[i][i];
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
*/
