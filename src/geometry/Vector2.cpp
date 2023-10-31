#include "geometry/main.hpp"

namespace geo
{
    const Vector2 Vector2::Origin = Vector2(0, 0);
	const Vector2 Vector2::Infinity = Vector2(std::numeric_limits<f64>::infinity(), std::numeric_limits<f64>::infinity());
	const Vector2 Vector2::iHat = Vector2(1, 0);
	const Vector2 Vector2::jHat = Vector2(0, 1);

	Vector2::Vector2() noexcept
	: x(0), y(0)
	{
	}

	Vector2::Vector2(const Vector2& v) noexcept
	: x(v.x), y(v.y)
	{
	}

	Vector2::Vector2(f64 x, f64 y) noexcept
	: x(x), y(y)
	{
	}

	f64& Vector2::operator[](size_t index)
	{
		assert(index <= 1);
		if (!index)
			return x;
		else;
			return y;
	}

	f64 Vector2::operator[](size_t index) const
	{
		assert(index <= 1);
		if (!index)
			return x;
		else
			return y;
	}

	Vector2 Vector2::Abs() const noexcept
	{
		return Vector2(geo::Abs(x), geo::Abs(y));
	}

	f64 Vector2::Angle(const Vector2& other) const noexcept
	{
		return acos(Dot(other) / (GetMagnitude() * other.GetMagnitude()));
	}

	f64 Vector2::Cross(const Vector2& v) const noexcept
	{
		return x * v.y - y * v.x;
	}

	Vector2 Vector2::Cross(const Vector2& v, f64 s) noexcept
	{
		return Vector2(s * v.y, -s * v.x);
	}

	Vector2 Vector2::Cross(f64 s, const Vector2& v) noexcept
	{
		return Vector2(-s * v.y, s * v.x);
	}

	f64 Vector2::Dot(const Vector2& v) const noexcept
	{
		return x * v.x + y * v.y;
	}

	f64 Vector2::GetMagnitude() const noexcept
	{
		if (!x && !y)
			return 0;
		return sqrt(x * x + y * y);
	}

	f64 Vector2::GetMagnitudeSquared() const noexcept
	{
		if (!x && !y)
			return 0;
		return x * x + y * y;
	}

	f64 Vector2::GetMagnitudeQuick() const noexcept
	{
		if (!x && !y)
			return 0;
		return FastSqrt(x * x + y * y);
	}

	Vector2 Vector2::Lerp(const Vector2& other, f64 t) const noexcept
	{
		return *this + (other - *this) * t;
	}

	void Vector2::Normalize() noexcept
	{
		f64 mag = GetMagnitude();
		if (mag > EPSILON)
			*this = *this / mag;
		else
			*this = Vector2(0, 0);
	}

	Vector2 Vector2::Normalized() const noexcept
	{
		Vector2 v = Vector2(this->x, this->y);
		v.Normalize();
		return v;
	}

	Vector2 operator*(f64 d, const Vector2& v) noexcept
	{
		return Vector2(d * v.x, d * v.y);
	}

	Vector2 operator+(f64 d, const Vector2& v) noexcept
	{
		return Vector2(d + v.x, d + v.y);
	}
	
	Vector2 operator-(f64 d, const Vector2& v) noexcept
	{
		return Vector2(d - v.x, d - v.y);
	}

	Vector2 operator/(f64 d, const Vector2& v) noexcept
	{
		return Vector2(d / v.x, d / v.y);
	}

	Vector2 Vector2::operator-() const noexcept
	{
		return Vector2(-x, -y);
	}

	Vector2 Vector2::operator+() const noexcept
	{
		return Vector2(+x, +y);
	}

	bool Vector2::operator==(const Vector2 &v) const noexcept
	{
		return x == v.x && y == v.y;
	}

	bool Vector2::operator!=(const Vector2 &v) const noexcept
	{
		return x != v.x || y != v.y;
	}

	Vector2 Vector2::operator+(const Vector2& v) const noexcept
	{
		Vector2 result(*this);
		result.x += v.x;
		result.y += v.y;
		return result;
	}

	Vector2 Vector2::operator+(f64 d) const noexcept
	{
		Vector2 result(*this);
		result.x += d;
		result.y += d;
		return result;
	}

	void Vector2::operator+=(const Vector2& v) noexcept
	{
		x += v.x;
		y += v.y;
	}

	void Vector2::operator+=(f64 d) noexcept
	{
		x += d;
		y += d;
	}

	Vector2 Vector2::operator-(const Vector2& v) const noexcept
	{
		Vector2 result(*this);
		result.x -= v.x;
		result.y -= v.y;
		return result;
	}

	Vector2 Vector2::operator-(f64 d) const noexcept
	{
		Vector2 result(*this);
		result.x -= d;
		result.y -= d;
		return result;
	}

	void Vector2::operator-=(const Vector2& v) noexcept
	{
		x -= v.x;
		y -= v.y;
	}

	void Vector2::operator-=(f64 d) noexcept
	{
		x -= d;
		y -= d;
	}

	Vector2 Vector2::operator*(const Vector2& v) const noexcept
	{
		Vector2 result(*this);
		result.x *= v.x;
		result.y *= v.y;
		return result;
	}

	Vector2 Vector2::operator*(f64 d) const noexcept
	{
		Vector2 result(*this);
		result.x *= d;
		result.y *= d;
		return result;
	}

	void Vector2::operator*=(const Vector2& v) noexcept
	{
		x *= v.x;
		y *= v.y;
	}

	void Vector2::operator*=(f64 d) noexcept
	{
		x *= d;
		y *= d;
	}

	Vector2 Vector2::operator/(const Vector2& v) const noexcept
	{
		Vector2 result(*this);
		result.x /= v.x;
		result.y /= v.y;
		return result;
	}

	Vector2 Vector2::operator/(f64 d) const noexcept
	{
		Vector2 result(*this);
		result.x /= d;
		result.y /= d;
		return result;
	}

	void Vector2::operator/=(const Vector2& v) noexcept
	{
		x /= v.x;
		y /= v.y;
	}

	void Vector2::operator/=(f64 d) noexcept
	{
		x /= d;
		y /= d;
	}

	bool Vector2::operator^(const Line& l) const noexcept
	{
		return (Line(l.a(), *this).angle() == Line(*this, l.b()).angle());
	}

	bool Vector2::operator<(const Vector2& v) const noexcept
	{
		return GetMagnitudeSquared() < v.GetMagnitudeSquared();
	}

	bool Vector2::operator>(const Vector2& v) const noexcept
	{
		return GetMagnitudeSquared() > v.GetMagnitudeSquared();
	}

	Vector2 Vector2::operator()() const noexcept
	{
		return Vector2(x, y);
	}

	void Vector2::Move(f64 offsetX, f64 offsetY) noexcept
	{
		this->x += offsetX;
		this->y += offsetY;
	}

	Vector2 Vector2::Projection(const Vector2& lhs, const Vector2& rhs) noexcept
	{
		double rhs_ls = rhs.GetMagnitudeSquared();
		return rhs * (rhs.Dot(lhs) / rhs_ls);
	}

	Vector2 Vector2::Projection(const Vector2& vector, const Line& target) noexcept
	{
		Vector2 AV = vector - target.a;
		Vector2 AB = target.b - target.a;
		return target.a + AV.Dot(AB) / AB.Dot(AB) * AB;
	}

	int Vector2::Quadrant(const Vector2& p) const noexcept
	{
		if (p.x < this->x)
		{
			if (p.y < this->y)
				return 3;
			else if (p.y > this->y)
				return 2;
			return 3;
		}
		else if (p.x > this->x)
		{
			if (p.y < this->y)
				return 4;
			else if (p.y > this->y)
				return 1;
			return 1;
		}
		return 0;
	}

	Vector2 Vector2::Reflection(const Vector2& normal) const noexcept
	{
		return *this - 2 * Dot(normal) * normal;
	}

	void Vector2::Reflect(const Vector2& normal) noexcept
	{
		*this = Reflection(normal);
	}

	void Vector2::Rotate(const Vector2& p, f64 angle) noexcept
	{
		f64 currentAngle = GetAngle(p, *this);
		f64 ang = angle;
		ang += currentAngle;
		if (ang < 0) {ang += (M_PI * 2);}
		if (ang > (M_PI) * 2) {ang -= M_PI;}
		f64 distance = Distance(*this, p);
		Vector2 v2 = GetVectorOnCircle(p, distance, ang);
		this->x = v2.x;
		this->y = v2.y;
	}

	void Vector2::Set(f64 newX, f64 newY) noexcept
	{
		this->x = newX;
		this->y = newY;
	}

	std::string Vector2::ToString() const noexcept
	{
		std::string strX = std::to_string(this->x);
		std::string strY = std::to_string(this->y);
		return "(" + strX + ", " + strY + ")";
	}
	
	std::tuple<f64, f64> Vector2::ToTuple() const noexcept
	{
		return std::tuple<f64, f64>(this->x, this->y);
	}

}