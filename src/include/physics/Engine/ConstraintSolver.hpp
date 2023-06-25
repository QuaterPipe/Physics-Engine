#include "../Dynamics/Constraint.hpp"
namespace physics
{
	geo::Vector ProjectedGaussianEliminationSolve(geo::Matrix matrix, geo::Vector right, f64 relaxation,
		i32 iterations, geo::Vector lo, geo::Vector hi);

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
				geo::Vector right;
				geo::Vector lo;
				geo::Vector high;
				f64 dt;
			};
	};
}