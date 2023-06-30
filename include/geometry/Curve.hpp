#pragma once
#include <algorithm>
#include "Vector.hpp"
namespace geo
{
	// \brief A cubic bezier curve.
	struct Curve
	{
		public:
			Vector2 a;
			Vector2 b;
			Vector2 c;
			Vector2 d;
			f64 t;
			Curve() noexcept;
			Curve(const Vector2& a, const Vector2& b,	const Vector2& c, const Vector2& d, const f64& t) noexcept;
			// \brief Returns a Vector on the curve based on the T-Value given
			Vector2 GetPoint(const f64& tValue = std::numeric_limits<double>::infinity()) const noexcept;
			// \brief Returns the derivative of the curve.
			Vector2 Derivative(const f64& tValue = std::numeric_limits<double>::infinity()) const noexcept;
	};
}