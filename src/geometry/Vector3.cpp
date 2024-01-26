#include "geometry/Vector.hpp"
#include "geometry/Math.hpp"

namespace geo
{
    const Vector3 Vector3::Origin = Vector3(0, 0, 0);
	const Vector3 Vector3::Infinity = Vector3(std::numeric_limits<f64>::infinity(), std::numeric_limits<f64>::infinity(), std::numeric_limits<f64>::infinity());
	const Vector3 Vector3::iHat = Vector3(1, 0, 0);
	const Vector3 Vector3::jHat = Vector3(0, 1, 0);
	const Vector3 Vector3::kHat = Vector3(0, 0, 1);

	Vector3::Vector3() noexcept
	: x(0), y(0), z(0)
	{
	}

	Vector3::Vector3(const Vector3& v) noexcept
	: x(v.x), y(v.y), z(v.z)
	{
	}

	Vector3::Vector3(f64 x, f64 y, f64 z) noexcept
	: x(x), y(y), z(z)
	{
	}

	f64& Vector3::operator[](size_t index)
	{
		assert(index <= 2);
		switch (index)
		{
			case 0:
				return x;
			case 1:
				return y;
			case 2:
				return z;
		}
		return x;
	}

	f64 Vector3::operator[](size_t index) const
	{
		assert(index <= 2);
		switch (index)
		{
			case 0:
				return x;
			case 1:
				return y;
			case 2:
				return z;
		}
		return x;
	}

	f64 Vector3::Dot(const Vector3& v) const noexcept
	{
		return x * v.x + y * v.y + z * v.z;
	}

	f64 Vector3::GetMagnitude() const noexcept
	{
		if (!x && !y && !z)
			return 0;
		return FastSqrt(x * x + y * y + z * z);
	}

	f64 Vector3::GetMagnitudeExact() const noexcept
	{
		if (!x && !y && !z)
			return 0;
		return sqrt(x * x + y * y + z * z);
	}

	f64 Vector3::GetMagnitudeSquared() const noexcept
	{
		if (!x && !y && !z)
			return 0;
		return x * x + y * y + z * z;
	}

	Vector3 Vector3::Lerp(const Vector3& other, f64 t) const noexcept
	{
		return *this + (other - *this) * t;
	}

	void Vector3::Normalize() noexcept
	{
		f64 mag = GetMagnitudeSquared();
		if (mag > SQRD(EPSILON))
			*this = *this * FastInvSqrt(mag);
		else
			*this = Vector3(0, 0, 0);
	}

	Vector3 Vector3::Normalized() const noexcept
	{
		Vector3 v = Vector3(*this);
		v.Normalize();
		return v;
	}

	Vector3 operator*(f64 d, const Vector3& v) noexcept
	{
		return Vector3(d * v.x, d * v.y, d * v.z);
	}

	Vector3 operator+(f64 d, const Vector3& v) noexcept
	{
		return Vector3(d + v.x, d + v.y, d + v.z);
	}
	
	Vector3 operator-(f64 d, const Vector3& v) noexcept
	{
		return Vector3(d - v.x, d - v.y, d - v.z);
	}

	Vector3 operator/(f64 d, const Vector3& v) noexcept
	{
		return Vector3(d / v.x, d / v.y, d / v.z);
	}

	Vector3 Vector3::operator-() const noexcept
	{
		return Vector3(-x, -y, -z);
	}

	Vector3 Vector3::operator+() const noexcept
	{
		return Vector3(+x, +y, +z);
	}

	bool Vector3::operator==(const Vector3 &v) const noexcept
	{
		return x == v.x && y == v.y && z == v.z;
	}

	bool Vector3::operator!=(const Vector3 &v) const noexcept
	{
		return x != v.x || y != v.y || z != v.z;
	}

	Vector3 Vector3::operator+(const Vector3& v) const noexcept
	{
		Vector3 result(x, y, z);
		result.x += v.x;
		result.y += v.y;
		result.z += v.z;
		return result;
	}

	Vector3 Vector3::operator+(f64 d) const noexcept
	{
		Vector3 result(x, y, z);
		result.x += d;
		result.y += d;
		result.z += d;
		return result;
	}

	void Vector3::operator+=(const Vector3& v) noexcept
	{
		x += v.x;
		y += v.y;
		z += v.z;
	}

	void Vector3::operator+=(f64 d) noexcept
	{
		x += d;
		y += d;
		z += d;
	}

	Vector3 Vector3::operator-(const Vector3& v) const noexcept
	{
		Vector3 result(x, y, z);
		result.x -= v.x;
		result.y -= v.y;
		result.z -= v.z;
		return result;
	}

	Vector3 Vector3::operator-(f64 d) const noexcept
	{
		Vector3 result(x, y, z);
		result.x -= d;
		result.y -= d;
		result.z -= d;
		return result;
	}

	void Vector3::operator-=(const Vector3& v) noexcept
	{
		x -= v.x;
		y -= v.y;
		z -= v.z;
	}

	void Vector3::operator-=(f64 d) noexcept
	{
		x -= d;
		y -= d;
		z -= d;
	}

	Vector3 Vector3::operator*(const Vector3& v) const noexcept
	{
		Vector3 result(x, y, z);
		result.x *= v.x;
		result.y *= v.y;
		result.z *= v.z;
		return result;
	}

	Vector3 Vector3::operator*(f64 d) const noexcept
	{
		Vector3 result(x, y, z);
		result.x *= d;
		result.y *= d;
		result.z *= d;
		return result;
	}

	void Vector3::operator*=(const Vector3& v) noexcept
	{
		x *= v.x;
		y *= v.y;
		z *= v.z;
	}

	void Vector3::operator*=(f64 d) noexcept
	{
		x *= d;
		y *= d;
		z *= d;
	}

	Vector3 Vector3::operator/(const Vector3& v) const noexcept
	{
		Vector3 result(x, y, z);
		result.x /= v.x;
		result.y /= v.y;
		result.z /= v.z;
		return result;
	}

	Vector3 Vector3::operator/(f64 d) const noexcept
	{
		Vector3 result(x, y, z);
		result.x /= d;
		result.y /= d;
		result.z /= d;
		return result;
	}

	void Vector3::operator/=(const Vector3& v) noexcept
	{
		x /= v.x;
		y /= v.y;
		z /= v.z;
	}

	void Vector3::operator/=(f64 d) noexcept
	{
		x /= d;
		y /= d;
		z /= d;
	}

	bool Vector3::operator<(const Vector3& v) const noexcept
	{
		return Distance(Vector3(0, 0, 0), v) > Distance(Vector3(0, 0, 0), *this);
	}

	bool Vector3::operator>(const Vector3& v) const noexcept
	{
		return Distance(Vector3(0, 0, 0), v) < Distance(Vector3(0, 0, 0), *this);
	}

	Vector3 Vector3::operator()() const noexcept
	{
		return Vector3(x, y, z);
	}

	void Vector3::Move(f64 offsetX, f64 offsetY, f64 offsetZ) noexcept
	{
		this->x += offsetX;
		this->y += offsetY;
		this->z += offsetZ;
	}

	Vector3 Vector3::Projection(const Vector3& lhs, const Vector3& rhs) noexcept
	{
		double rhs_ls = rhs.GetMagnitudeSquared();
		return rhs * (rhs.Dot(lhs) / rhs_ls);
	}

	std::string Vector3::ToString() const noexcept
	{
		std::string strX = std::to_string(this->x);
		std::string strY = std::to_string(this->y);
		std::string strZ = std::to_string(this->z);
		return "(" + strX + ", " + strY + ", " + strZ + ")";
	}
	
	std::tuple<f64, f64, f64> Vector3::ToTuple() const noexcept
	{
		return std::tuple<f64, f64, f64>(this->x, this->y, this->z);
	}

	void Vector3::Set(f64 newX, f64 newY, f64 newZ) noexcept
	{
		x = newX;
		y = newY;
		z = newZ;
	}
}