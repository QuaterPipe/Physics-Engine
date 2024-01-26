#pragma once
#include "physics/Dynamics/Dynamicbody.hpp"
namespace physics
{
	class Constraint
	{
		public:
			static constexpr int MaxConstraintCount = 3;
			static constexpr int MaxBodyCount = 2;
			struct Output
			{
				f64 C[MaxConstraintCount];
				f64 J[MaxConstraintCount][3 * MaxBodyCount];
				f64 J_dot[MaxConstraintCount][3 * MaxBodyCount];
				f64 v_bias[MaxConstraintCount];
				f64 limits[MaxConstraintCount][2];
				f64 ks[MaxConstraintCount];
				f64 kd[MaxConstraintCount];
			};

			f64 Fx[MaxConstraintCount][MaxBodyCount];
			f64 Fy[MaxConstraintCount][MaxBodyCount];
			f64 Ft[MaxConstraintCount][MaxBodyCount];

			Constraint(int constraintCount, int bodyCount) noexcept;
			virtual ~Constraint() noexcept;
			virtual void Calculate(Output* output) noexcept;
			size_t bodyCount = 0;
			Dynamicbody* bodies[MaxBodyCount];
		protected:
			inline void noLimits(Output* output) {
				for (int i = 0; i < MaxConstraintCount; ++i) {
					output->limits[i][0] = -DBL_MAX;
					output->limits[i][1] = DBL_MAX;
				}
			}
			size_t constraintCount;
	};
}