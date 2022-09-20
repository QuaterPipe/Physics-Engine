#include "Dynamicbody.hpp"
namespace physics
{	
	struct Constraint
	{
		Dynamicbody* a;
		Dynamicbody* b;
		struct Output
		{
			geo::Matrix K; // J * M^-1 * J^T 
			geo::Matrix J;
			geo::Matrix Limits;
			geo::Vector B; // bias
		};
		virtual Output Calculate(f64 dt) = 0;
		size_t constraintCount = 1;
	};
}