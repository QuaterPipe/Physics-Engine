#pragma once
#include "../../geometry/main.hpp"
namespace physics
{	
	struct Dynamicbody;
	struct Constraint
	{
		Dynamicbody* a;
		Dynamicbody* b;
		struct Output
		{
			geo::Matrix K; // J * M^-1 * J^T 
			geo::Matrix J;
			geo::Vector low;
			geo::Vector high;
			geo::Vector rhs; // -(J * (v + M^-1 * Fext * dt) + b)
			geo::Vector B; // bias
			geo::Vector Q;
			geo::Vector lambda;

		};
		virtual Output Calculate(f64 dt) = 0;
		size_t constraintLength = 1;
		Output lastFrame;
	};
}