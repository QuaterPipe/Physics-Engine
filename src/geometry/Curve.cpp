#include "geometry/Curve.hpp"

namespace geo
{
	Curve::Curve() noexcept
	{
		t = 0;
		a.Set(0, 0);
		b.Set(0, 1);
		c.Set(1, 1);
		d.Set(1, 0);
	}

	Curve::Curve(const Vector2& a, const Vector2& b, const Vector2& c, const Vector2& d, const f64& t) noexcept
	{
		this->a = a;
		this->b = b;
		this->c = c;
		this->d = d;
		this->t = t;
	}

	Vector2 Curve::GetPoint(const f64& tValue) const noexcept
	{
		if (tValue == std::numeric_limits<double>::infinity())
		{
			return a.Lerp(b, t).Lerp(b.Lerp(c, t), t).Lerp(b.Lerp(c, t).Lerp(c.Lerp(d, t), t), t);
		}
		return a.Lerp(b, tValue).Lerp(b.Lerp(c, tValue), tValue).Lerp(b.Lerp(c, tValue).Lerp(c.Lerp(d, tValue), tValue), tValue);
	}

	Vector2 Curve::Derivative(const f64& tValue) const noexcept
	{
		if (tValue == std::numeric_limits<double>::infinity())
		{
			Vector2 deriv = a * (-3 * (t * t) + 6 * t - 3) + 
				b * (9 * (t * t) -12 * t + 3) +
				c * (-9 * (t * t) + 6 * t) +
				d * (3 * (t * t));
			return deriv;
		}
		Vector2 deriv = a * (-3 * (tValue * tValue) + 6 * t - 3) + 
			b * (9 * (tValue * tValue) -12 * tValue + 3) +
			c * (-9 * (tValue * tValue) + 6 * tValue) +
			d * (3 * (tValue * tValue));
		return deriv;
	}
}