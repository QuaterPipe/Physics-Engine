#pragma once
#include "../Geometry/Vector.hpp"

namespace physics
{
	struct RK4Step
	{
		const static int STEP1 = 0;
		const static int STEP2 = 1;
		const static int STEP3 = 2;
		const static int STEP4 = 3;
	};

	struct RK4State
	{
		Vector2 k1X, k2X, k3X, k4X;
		Vector2 k1V, k2V, k3V, k4V;
		Vector2 a1, a2, a3, a4;
		Vector2 tmpX, tmpV;

		RK4State();
		void Reset() noexcept;
	};
}