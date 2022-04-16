#include "../include/geometry/Math.hpp"
#include "../include/geometry/Line.hpp"
#include "../include/geometry/Vector.hpp"

namespace geometry
{

	const Vector Vector::Origin = Vector(0, 0);
	const Vector Vector::Infinity = Vector(std::numeric_limits<f64>::infinity(), std::numeric_limits<f64>::infinity());
	const Vector Vector::iHat = Vector(1, 0);
	const Vector Vector::jHat = Vector(0, 1);

	Vector::Vector() noexcept
	: x(0), y(0)
	{
	}

	Vector::Vector(const f64& x, const f64& y) noexcept
	: x(x), y(y)
	{
	}

	Vector Vector::Abs() const noexcept
	{
		return Vector(fabs(x), fabs(y));
	}

	f64 Vector::Angle(const Vector& other) const noexcept
	{
		return acos(Dot(other) / (GetMagnitude() * other.GetMagnitude()));
	}

	f64 Vector::Cross(const Vector& v) const noexcept
	{
		return x * v.y - y * v.x;
	}

	Vector Vector::Cross(const Vector& v, const f64& s) noexcept
	{
		return Vector(s * v.y, -s * v.x);
	}

	Vector Vector::Cross(const f64& s, const Vector& v) noexcept
	{
		return Vector(-s * v.y, s * v.x);
	}

	f64 Vector::Dot(const Vector& v) const noexcept
	{
		return x * v.x + y * v.y;
	}

	f64 Vector::GetMagnitude() const noexcept
	{
		return sqrt(x * x + y * y);
	}

	f64 Vector::GetMagnitudeSquared() const noexcept
	{
		return x * x + y * y;
	}

	f64 Vector::GetMagnitudeQuick() const noexcept
	{
		return FastSqrt(x * x + y * y);
	}

	Vector Vector::Lerp(const Vector& other, const f64& t) const noexcept
	{
		return *this + (other - *this) * t;
	}

	void Vector::Normalize() noexcept
	{
		f64 mag = GetMagnitude();
		if (mag > 0.00001)
			*this = *this / mag;
		else
			*this = Vector(0, 0);
	}

	Vector Vector::Normalized() const noexcept
	{
		Vector v = Vector(this->x, this->y);
		v.Normalize();
		return v;
	}

	Vector operator*(const f64 & d, const Vector& v) noexcept
	{
		return Vector(d * v.x, d * v.y);
	}

	Vector operator+(const f64 & d, const Vector& v) noexcept
	{
		return Vector(d + v.x, d + v.y);
	}

	Vector Vector::operator-() const noexcept
	{
		return Vector(-x, -y);
	}

	Vector Vector::operator+() const noexcept
	{
		return Vector(+x, +y);
	}

	bool Vector::operator==(const Vector &v) const noexcept
	{
		return x == v.x && y == v.y;
	}

	bool Vector::operator!=(const Vector &v) const noexcept
	{
		return x != v.x || y != v.y;
	}

	Vector Vector::operator+(const Vector& v) const noexcept
	{
		Vector result(*this);
		result.x += v.x;
		result.y += v.y;
		return result;
	}

	Vector Vector::operator+(const f64& d) const noexcept
	{
		Vector result(*this);
		result.x += d;
		result.y += d;
		return result;
	}

	void Vector::operator+=(const Vector& v) noexcept
	{
		x += v.x;
		y += v.y;
	}

	void Vector::operator+=(const f64& d) noexcept
	{
		x += d;
		y += d;
	}

	Vector Vector::operator-(const Vector& v) const noexcept
	{
		Vector result(*this);
		result.x -= v.x;
		result.y -= v.y;
		return result;
	}

	Vector Vector::operator-(const f64& d) const noexcept
	{
		Vector result(*this);
		result.x -= d;
		result.y -= d;
		return result;
	}

	void Vector::operator-=(const Vector& v) noexcept
	{
		x -= v.x;
		y -= v.y;
	}

	void Vector::operator-=(const f64& d) noexcept
	{
		x -= d;
		y -= d;
	}

	Vector Vector::operator*(const Vector& v) const noexcept
	{
		Vector result(*this);
		result.x *= v.x;
		result.y *= v.y;
		return result;
	}

	Vector Vector::operator*(const f64& d) const noexcept
	{
		Vector result(*this);
		result.x *= d;
		result.y *= d;
		return result;
	}

	void Vector::operator*=(const Vector& v) noexcept
	{
		x *= v.x;
		y *= v.y;
	}

	void Vector::operator*=(const f64& d) noexcept
	{
		x *= d;
		y *= d;
	}

	Vector Vector::operator/(const Vector& v) const noexcept
	{
		Vector result(*this);
		result.x /= v.x;
		result.y /= v.y;
		return result;
	}

	Vector Vector::operator/(const f64& d) const noexcept
	{
		Vector result(*this);
		result.x /= d;
		result.y /= d;
		return result;
	}

	void Vector::operator/=(const Vector& v) noexcept
	{
		x /= v.x;
		y /= v.y;
	}

	void Vector::operator/=(const f64& d) noexcept
	{
		x /= d;
		y /= d;
	}

	bool Vector::operator^(const Line& l) const noexcept
	{
		return (Line(l.a(), *this).angle() == Line(*this, l.b()).angle());
	}

	bool Vector::operator<(const Vector& v) const noexcept
	{
		return Distance(Origin, v) > Distance(Origin, *this);
	}

	bool Vector::operator>(const Vector& v) const noexcept
	{
		return Distance(Origin, v) < Distance(Origin, *this);
	}

	Vector Vector::operator()() const noexcept
	{
		return Vector(x, y);
	}

	void Vector::Move(const f64& offsetX, const f64& offsetY) noexcept
	{
		this->x += offsetX;
		this->y += offsetY;
	}

	Vector Vector::Projection(const Vector& lhs, const Vector& rhs) noexcept
	{
		double rhs_ls = rhs.GetMagnitudeSquared();
		return rhs * (rhs.Dot(lhs) / rhs_ls);
	}

	Vector Vector::Projection(const Vector& vector, const Line& target) noexcept
	{
		Vector AV = vector - target.a;
		Vector AB = target.b - target.a;
		return target.a + AV.Dot(AB) / AB.Dot(AB) * AB;
	}

	int Vector::Quadrant(const Vector& p) const noexcept
	{
		if (p.x < this->x)
		{
			if (p.y < this->y)
			{
				return 3;
			}
			else if (p.y > this->y)
			{
				return 2;
			}
			return 3;
		}
		else if (p.x > this->x)
		{
			if (p.y < this->y)
			{
				return 4;
			}
			else if (p.y > this->y)
			{
				return 1;
			}
			return 1;
		}
		return 0;
	}

	Vector Vector::Reflection(const Vector& normal) const noexcept
	{
		return *this - 2 * Dot(normal) * normal;
	}

	void Vector::Reflect(const Vector& normal) noexcept
	{
		*this = Reflection(normal);
	}

	void Vector::Rotate(const Vector& p, const f64& angle) noexcept
	{
		f64 currentAngle = GetAngle(p, *this);
		f64 ang = angle;
		ang += currentAngle;
		if (ang < 0) {ang += (M_PI * 2);}
		if (ang > (M_PI) * 2) {ang -= M_PI;}
		f64 distance = Distance(*this, p);
		Vector v2 = GetVectorOnCircle(p, distance, ang);
		this->x = v2.x;
		this->y = v2.y;
	}

	void Vector::Set(const f64& newX, const f64& newY) noexcept
	{
		this->x = newX;
		this->y = newY;
	}

	std::string Vector::ToString() const noexcept
	{
		std::string strX = std::to_string(this->x);
		std::string strY = std::to_string(this->y);
		return "(" + strX + ", " + strY + ")";
	}
	
	std::tuple<f64, f64> Vector::ToTuple() const noexcept
	{
		return std::tuple<f64, f64>(this->x, this->y);
	}


	const Vector3 Vector3::Origin = Vector3(0, 0, 0);
	const Vector3 Vector3::Infinity = Vector3(std::numeric_limits<f64>::infinity(), std::numeric_limits<f64>::infinity(), std::numeric_limits<f64>::infinity());
	const Vector3 Vector3::iHat = Vector3(1, 0, 0);
	const Vector3 Vector3::jHat = Vector3(0, 1, 0);
	const Vector3 Vector3::kHat = Vector3(0, 0, 1);

	Vector3::Vector3() noexcept
	{
		x = 0;
		y = 0;
		z = 0;
	}

	Vector3::Vector3(const f64& x, const f64& y, const f64& z) noexcept
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}

	f64 Vector3::Dot(const Vector3& v) const noexcept
	{
		return x * v.x + y * v.y + z * v.z;
	}

	f64 Vector3::GetMagnitude() const noexcept
	{
		return sqrt(x * x + y * y + z * z);
	}

	f64 Vector3::GetMagnitudeSquared() const noexcept
	{
		return x * x + y * y + z * z;
	}

	f64 Vector3::GetMagnitudeQuick() const noexcept
	{
		return FastSqrt(x * x + y * y + z * z);
	}

	Vector3 Vector3::Lerp(const Vector3& other, const f64& t) const noexcept
	{
		return *this + (other - *this) * t;
	}

	void Vector3::Normalize() noexcept
	{
		f64 mag = GetMagnitude();
		if (mag > 0.00001)
			*this = *this / mag;
		else
			*this = Vector3(0, 0, 0);
	}

	Vector3 Vector3::Normalized() const noexcept
	{
		Vector3 v = Vector3(this->x, this->y, this->z);
		v.Normalize();
		return v;
	}

	Vector3 operator*(const f64& d, const Vector3& v) noexcept
	{
		return Vector3(d * v.x, d * v.y, d * v.z);
	}

	Vector3 operator+(const f64& d, const Vector3& v) noexcept
	{
		return Vector3(d + v.x, d + v.y, d + v.z);
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
		Vector3 result(*this);
		result.x += v.x;
		result.y += v.y;
		result.z += v.z;
		return result;
	}

	Vector3 Vector3::operator+(const f64& d) const noexcept
	{
		Vector3 result(*this);
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

	void Vector3::operator+=(const f64& d) noexcept
	{
		x += d;
		y += d;
		z += d;
	}

	Vector3 Vector3::operator-(const Vector3& v) const noexcept
	{
		Vector3 result(*this);
		result.x -= v.x;
		result.y -= v.y;
		result.z -= v.z;
		return result;
	}

	Vector3 Vector3::operator-(const f64& d) const noexcept
	{
		Vector3 result(*this);
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

	void Vector3::operator-=(const f64& d) noexcept
	{
		x -= d;
		y -= d;
		z -= d;
	}

	Vector3 Vector3::operator*(const Vector3& v) const noexcept
	{
		Vector3 result(*this);
		result.x *= v.x;
		result.y *= v.y;
		result.z *= v.z;
		return result;
	}

	Vector3 Vector3::operator*(const f64& d) const noexcept
	{
		Vector3 result(*this);
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

	void Vector3::operator*=(const f64& d) noexcept
	{
		x *= d;
		y *= d;
		z *= d;
	}

	Vector3 Vector3::operator/(const Vector3& v) const noexcept
	{
		Vector3 result(*this);
		result.x /= v.x;
		result.y /= v.y;
		result.z /= v.z;
		return result;
	}

	Vector3 Vector3::operator/(const f64& d) const noexcept
	{
		Vector3 result(*this);
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

	void Vector3::operator/=(const f64& d) noexcept
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

	void Vector3::Move(const f64& offsetX, const f64& offsetY, const f64& offsetZ) noexcept
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

	void Vector3::Set(const f64& newX, const f64& newY, const f64& newZ) noexcept
	{
		x = newX;
		y = newY;
		z = newZ;
	}
}