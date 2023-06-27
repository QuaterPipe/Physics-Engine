#include "../include/geometry/Math.hpp"
#include "../include/geometry/Line.hpp"
#include "../include/geometry/Vector.hpp"

namespace geo
{
	Vector::Vector() noexcept
	{
	}

	Vector::Vector(size_t size, f64 n) noexcept
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