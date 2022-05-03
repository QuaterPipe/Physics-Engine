#pragma once
#include <algorithm>
#include "Vector.hpp"
namespace geo
{
	// \brief A cubic bezier curve.
	struct Curve
	{
		public:
			Vector a;
			Vector b;
			Vector c;
			Vector d;
			f64 t;
			Curve() noexcept;
			Curve(const Vector& a, const Vector& b,	const Vector& c, const Vector& d, const f64& t) noexcept;
			// \brief Returns a Vector on the curve based on the T-Value given
			Vector GetPoint(const f64& tValue = std::numeric_limits<double>::infinity()) const noexcept;
			// \brief Returns the derivative of the curve.
			Vector Derivative(const f64& tValue = std::numeric_limits<double>::infinity()) const noexcept;
	};
}