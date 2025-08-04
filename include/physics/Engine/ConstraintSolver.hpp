#pragma once
#include "physics/Dynamics/Constraint.hpp"
namespace physics
{
	Vector ProjectedGaussianEliminationSolve(Matrix matrix, Vector right, f64 relaxation,
		i32 iterations, Vector lo, Vector hi);

	class ConstraintSolver
	{
		public:
			struct BlockSparseMatrix;
			struct BlockSparseMatrix
			{
				BlockSparseMatrix(const std::vector<Constraint*>& constraints);
				struct Section
				{
					Section(Constraint* constraint);
					f64 operator[](size_t index);
					Constraint* constraint;
				};
				Section operator[](size_t index);
				std::vector<Section> sections;
			};
			struct IntermediateValues
			{
				BlockSparseMatrix K;
				Vector right;
				Vector lo;
				Vector high;
				f64 dt;
			};
	};
}