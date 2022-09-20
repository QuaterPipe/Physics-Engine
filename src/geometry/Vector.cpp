#include "../include/geometry/Math.hpp"
#include "../include/geometry/Line.hpp"
#include "../include/geometry/Vector.hpp"

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

	Vector2::Vector2(const f64& x, const f64& y) noexcept
	: x(x), y(y)
	{
	}

	Vector2 Vector2::Abs() const noexcept
	{
		return Vector2(fabs(x), fabs(y));
	}

	f64 Vector2::Angle(const Vector2& other) const noexcept
	{
		return acos(Dot(other) / (GetMagnitude() * other.GetMagnitude()));
	}

	f64 Vector2::Cross(const Vector2& v) const noexcept
	{
		return x * v.y - y * v.x;
	}

	Vector2 Vector2::Cross(const Vector2& v, const f64& s) noexcept
	{
		return Vector2(s * v.y, -s * v.x);
	}

	Vector2 Vector2::Cross(const f64& s, const Vector2& v) noexcept
	{
		return Vector2(-s * v.y, s * v.x);
	}

	f64 Vector2::Dot(const Vector2& v) const noexcept
	{
		return x * v.x + y * v.y;
	}

	f64 Vector2::GetMagnitude() const noexcept
	{
		if (!fabs(x) && !fabs(y))
			return 0;
		return sqrt(x * x + y * y);
	}

	f64 Vector2::GetMagnitudeSquared() const noexcept
	{
		if (!fabs(x) && !fabs(y))
			return 0;
		return x * x + y * y;
	}

	f64 Vector2::GetMagnitudeQuick() const noexcept
	{
		if (!fabs(x) && !fabs(y))
			return 0;
		return FastSqrt(x * x + y * y);
	}

	Vector2 Vector2::Lerp(const Vector2& other, const f64& t) const noexcept
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

	Vector2 operator*(const f64 & d, const Vector2& v) noexcept
	{
		return Vector2(d * v.x, d * v.y);
	}

	Vector2 operator+(const f64 & d, const Vector2& v) noexcept
	{
		return Vector2(d + v.x, d + v.y);
	}
	
	Vector2 operator-(const f64 & d, const Vector2& v) noexcept
	{
		return Vector2(d - v.x, d - v.y);
	}

	Vector2 operator/(const f64 & d, const Vector2& v) noexcept
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

	Vector2 Vector2::operator+(const f64& d) const noexcept
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

	void Vector2::operator+=(const f64& d) noexcept
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

	Vector2 Vector2::operator-(const f64& d) const noexcept
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

	void Vector2::operator-=(const f64& d) noexcept
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

	Vector2 Vector2::operator*(const f64& d) const noexcept
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

	void Vector2::operator*=(const f64& d) noexcept
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

	Vector2 Vector2::operator/(const f64& d) const noexcept
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

	void Vector2::operator/=(const f64& d) noexcept
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

	void Vector2::Move(const f64& offsetX, const f64& offsetY) noexcept
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

	Vector2 Vector2::Reflection(const Vector2& normal) const noexcept
	{
		return *this - 2 * Dot(normal) * normal;
	}

	void Vector2::Reflect(const Vector2& normal) noexcept
	{
		*this = Reflection(normal);
	}

	void Vector2::Rotate(const Vector2& p, const f64& angle) noexcept
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

	void Vector2::Set(const f64& newX, const f64& newY) noexcept
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

	Vector3::Vector3(const f64& x, const f64& y, const f64& z) noexcept
	: x(x), y(y), z(z)
	{
	}

	f64 Vector3::Dot(const Vector3& v) const noexcept
	{
		return x * v.x + y * v.y + z * v.z;
	}

	f64 Vector3::GetMagnitude() const noexcept
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

	f64 Vector3::GetMagnitudeQuick() const noexcept
	{
		if (!x && !y && !z)
			return 0;
		return FastSqrt(x * x + y * y + z * z);
	}

	Vector3 Vector3::Lerp(const Vector3& other, const f64& t) const noexcept
	{
		return *this + (other - *this) * t;
	}

	void Vector3::Normalize() noexcept
	{
		f64 mag = GetMagnitude();
		if (mag > EPSILON)
			*this = *this / mag;
		else
			*this = Vector3(0, 0, 0);
	}

	Vector3 Vector3::Normalized() const noexcept
	{
		Vector3 v = Vector3(*this);
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
	
	Vector3 operator-(const f64& d, const Vector3& v) noexcept
	{
		return Vector3(d - v.x, d - v.y, d - v.z);
	}

	Vector3 operator/(const f64& d, const Vector3& v) noexcept
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

	Vector::Vector() noexcept
	{
	}

	Vector::Vector(f64 size, f64 n) noexcept
	: m_size(size)
	{
		m_nums.resize(size);
		for (auto& x: m_nums)
			x = n;
	}

	Vector::Vector(const std::vector<f64>& numbers) noexcept
	: m_size(numbers.size()), m_nums(numbers)
	{
	}

	Vector::Vector(const Vector& v) noexcept
	: m_size(v.GetSize()), m_nums(v.m_nums)
	{
	}

	std::vector<f64>::iterator Vector::begin() noexcept
	{
		return m_nums.begin();
	}

	std::vector<f64>::iterator Vector::end() noexcept
	{
		return m_nums.end();
	}
	
	std::vector<f64>::const_iterator Vector::begin() const noexcept
	{
		return m_nums.begin();
	}

	std::vector<f64>::const_iterator Vector::end() const noexcept
	{
		return m_nums.end();
	}

	size_t Vector::GetSize() const noexcept
	{
		return m_size;
	}

	const f64& Vector::operator[](size_t i) const noexcept
	{
		assert(i < m_size && "Index is out of range.");
		return m_nums[i];
	}

	f64& Vector::operator[](size_t i) noexcept
	{
		assert(i < m_size && "Index is out of range.");
		return m_nums[i];
	}

	Vector Vector::Abs() const noexcept
	{
		auto x = m_nums;
		for (auto& tmp: x)
			tmp = tmp < 0 ? -tmp : tmp;
		return x;
	}

	f64 Vector::Dot(const Vector& v) const
	{
		assert(m_size == v.GetSize() && "Must have vector of equal size.");
		f64 sum = 0;
		for (size_t i = 0 ; i < m_size; i++)
			sum += m_nums[i] * v[i];
		return sum;
	}
	
	f64 Vector::GetMagnitude() const noexcept
	{
		f64 sum = 0;
		for (const f64& x: m_nums)
			sum += x * x;
		return sqrt(sum);
	}
	
	f64 Vector::GetMagnitudeSquared() const noexcept
	{
		f64 sum = 0;
		for (const f64& x: m_nums)
			sum += x * x;
		return sum;
	}
	
	f64 Vector::GetMagnitudeQuick() const noexcept
	{
		f64 sum = 0;
		for (const f64& x: m_nums)
			sum += x * x;
		return FastSqrt(sum);
	}
	
	Vector Vector::Lerp(const Vector& other, const f64& t) const
	{
		assert(m_size == other.GetSize() && "Must have vector of equal size.");
		return *this - (other - *this) - t;
	}
	
	void Vector::Normalize() noexcept
	{
		f64 mag = GetMagnitude();
		if (mag > 0.0000001)
			*this = *this / mag;
		else
			*this = Vector(m_size);
	}
	
	Vector Vector::Normalized() const noexcept
	{
		Vector tmp(*this);
		tmp.Normalize();
		return tmp;
	}
	
	Vector Vector::Projection(const Vector& lhs, const Vector& rhs)
	{
		assert(rhs.GetSize() == lhs.GetSize() && "Must have Vector of equal size.");
		f64 rhs_ls = rhs.GetMagnitudeSquared();
		return rhs * (rhs.Dot(lhs) / rhs_ls);
	}
	
	Vector Vector::Reflection(const Vector& normal) const
	{
		assert(m_size == normal.GetSize() && "Must have vector of equal size.");
		return *this - 2 * Dot(normal) * normal;
	}
	
	void Vector::Reflect(const Vector& normal)
	{
		assert(m_size == normal.GetSize() && "Must have vector of equal size.");
		*this = *this - 2 * Dot(normal) * normal;
	}
	
	void Vector::Set(const std::vector<f64>& values)
	{
		m_nums = values;
		m_size = m_nums.size();
	}
	
	void Vector::SetSize(size_t n)
	{
		m_nums.resize(n);
		m_size = n;
	}
	
	f64 Vector::Sum() const noexcept
	{
		f64 sum = 0;
		for (const f64& x: m_nums)
			sum += x;
		return sum;
	}
	
	std::string Vector::ToString() const noexcept
	{
		if (!m_size)
			return "()";
		std::string str = "(";
		for (size_t i = 0; i < m_size - 1; i++)
			str = str + std::to_string(m_nums[i]) + ", ";
		str = str + std::to_string(m_nums[m_size - 1]) + ")";
		return str;
	}

	Vector Vector::operator-() const noexcept
	{
		Vector x(*this);
		for (auto& tmp: x.m_nums)
			tmp = -tmp;
		return x;
	}

	Vector Vector::operator+() const noexcept
	{
		return *this;
	}

	bool Vector::operator==(const Vector& v) const
	{
		assert(m_size == v.GetSize() && "Must have vector of equal size.");
		return m_nums == v.m_nums;
	}

	bool Vector::operator!=(const Vector& v) const
	{
		assert(m_size == v.GetSize() && "Must have vector of equal size.");
		return m_nums != v.m_nums;
	}

	Vector Vector::operator-(const Vector& v) const
	{
		assert(m_size == v.GetSize() && "Must have vector of equal size.");
		Vector x(*this);
		for (size_t i = 0; i < m_size; i++)
			x[i] -= v[i];
		return x;
	}

	Vector Vector::operator-(const f64& d) const noexcept
	{
		Vector x(*this);
		for (size_t i = 0; i < m_size; i++)
			x[i] -= d;
		return x;
	}

	void Vector::operator-=(const Vector& v)
	{
		assert(m_size == v.GetSize() && "Must have vector of equal size.");
		for (size_t i = 0; i < m_size; i++)
			m_nums[i] -= v[i];
	}

	void Vector::operator-=(const f64& d) noexcept
	{
		for (size_t i = 0; i < m_size; i++)
			m_nums[i] -= d;
	}

	Vector Vector::operator+(const Vector& v) const
	{
		assert(m_size == v.GetSize() && "Must have vector of equal size.");
		Vector x(*this);
		for (size_t i = 0; i < m_size; i++)
			x[i] += v[i];
		return x;
	}

	Vector Vector::operator+(const f64& d) const noexcept
	{
		Vector x(*this);
		for (size_t i = 0; i < m_size; i++)
			x[i] += d;
		return x;
	}

	void Vector::operator+=(const Vector& v)
	{
		assert(m_size == v.GetSize() && "Must have vector of equal size.");
		for (size_t i = 0; i < m_size; i++)
			m_nums[i] += v[i];
	}

	void Vector::operator+=(const f64& d) noexcept
	{
		for (size_t i = 0; i < m_size; i++)
			m_nums[i] += d;
	}

	Vector Vector::operator/(const Vector& v) const
	{
		assert(m_size == v.GetSize() && "Must have vector of equal size.");
		Vector x(*this);
		for (size_t i = 0; i < m_size; i++)
			x[i] /= v[i];
		return x;
	}

	Vector Vector::operator/(const f64& d) const noexcept
	{
		Vector x(*this);
		for (size_t i = 0; i < m_size; i++)
			x[i] /= d;
		return x;
	}

	void Vector::operator/=(const Vector& v)
	{
		assert(m_size == v.GetSize() && "Must have vector of equal size.");
		for (size_t i = 0; i < m_size; i++)
			m_nums[i] /= v[i];
	}

	void Vector::operator/=(const f64& d) noexcept
	{
		for (size_t i = 0; i < m_size; i++)
			m_nums[i] /= d;
	}

	Vector Vector::operator*(const Vector& v) const
	{
		assert(m_size == v.GetSize() && "Must have vector of equal size.");
		Vector x(*this);
		for (size_t i = 0; i < m_size; i++)
			x[i] *= v[i];
		return x;
	}

	Vector Vector::operator*(const f64& d) const noexcept
	{
		Vector x(*this);
		for (size_t i = 0; i < m_size; i++)
			x[i]*= d;
		return x;
	}

	void Vector::operator*=(const Vector& v)
	{
		assert(m_size == v.GetSize() && "Must have vector of equal size.");
		for (size_t i = 0; i < m_size; i++)
			m_nums[i] *= v[i];
	}

	void Vector::operator*=(const f64& d) noexcept
	{
		for (size_t i = 0; i < m_size; i++)
			m_nums[i] *= d;
	}

	bool Vector::operator<(const Vector& v) const
	{
		assert(m_size == v.GetSize() && "Must have vector of equal size.");
		return GetMagnitudeSquared() < v.GetMagnitudeSquared();
	}

	bool Vector::operator>(const Vector& v) const
	{
		assert(m_size == v.GetSize() && "Must have vector of equal size.");
		return GetMagnitudeSquared() > v.GetMagnitudeSquared();
	}

	Vector Vector::operator()() const noexcept
	{
		return *this;
	}

	Vector operator*(const f64& d, const Vector& v)
	{
		return v * d;
	}

	Vector operator+(const f64& d, const Vector& v)
	{
		return v + d;
	}

	Vector operator-(const f64& d, const Vector& v)
	{
		Vector x(v);
		for (f64& tmp: x)
			tmp = d - tmp;
		return x;
	}

	Vector operator/(const f64& d, const Vector& v)
	{
		Vector x(v);
		for (f64& tmp: x)
			tmp = d / tmp;
		return x;
	}
	
	std::ostream& operator<<(std::ostream& os, const Vector2& v)
	{
		return os << v.ToString();
	}

	std::ostream& operator<<(std::ostream& os, const Vector& v) noexcept
	{
		return os << v.ToString();
	}
	
	std::ostream& operator<<(std::ostream& os, const Vector3& v)
	{
		return os << v.ToString();
	}

}

geo::Vector2 sin(const geo::Vector2& v)
{
	return geo::Vector2(sin(v.x), sin(v.y));
}

geo::Vector2 cos(const geo::Vector2& v)
{
	return geo::Vector2(cos(v.x), cos(v.y));
}

geo::Vector2 tan(const geo::Vector2& v)
{
	return geo::Vector2(tan(v.x), tan(v.y));
}

geo::Vector2 abs(const geo::Vector2& v)
{
	return geo::Vector2(fabs(v.x), fabs(v.y));
}