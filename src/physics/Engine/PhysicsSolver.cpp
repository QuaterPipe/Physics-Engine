#include "../../include/physics/Engine/DynamicsWorld.hpp"
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
	
	void PhysicsSolver::Solve(std::vector<Collision>& collisions, f64 dt) noexcept
	{
		for (Collision& c: collisions)
		{
			if (!c.a->IsDynamic() || !c.b->IsDynamic()) continue;
			Dynamicbody* a = dynamic_cast<Dynamicbody*>(c.a);
			Dynamicbody* b = dynamic_cast<Dynamicbody*>(c.b);
			if (!a || !b) return;
			
		}
	}
}