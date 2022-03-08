#include "../include/geometry/Matrix.hpp"

namespace geometry
{
	Matrix2::Matrix2() noexcept
	{
		a = 1;
		d = 1;
	}

	Matrix2::Matrix2(const f64& radians) noexcept
	{
		a = cos(radians);
		b = -sin(radians);
		c = sin(radians);
		d = cos(radians);
	}

	Matrix2::Matrix2(const f64& a, const f64& b, const f64& c, const f64& d) noexcept
	{
		this->a = a;
		this->b = b;
		this->c = c;
		this->d = d;
	}

	Matrix2& Matrix2::operator=(const Matrix2& other) noexcept
	{
		this->a = other.a;
		this->b = other.b;
		this->c = other.c;
		this->d = other.d;
		return *this;
	}

	Vector Matrix2::AxisX() const noexcept
	{
		return Vector(a, c);
	}

	Vector Matrix2::AxisY() const noexcept
	{
		return Vector(b, d);
	}

	f64 Matrix2::Determinant() const noexcept
	{
		return a * d - b * c;
	}

	Matrix2 Matrix2::Transpose() const noexcept
	{
		return Matrix2(a, c, b, d);
	}

	Vector Matrix2::operator*(const Vector& v) const noexcept
	{
		return Vector(a * v.x + b * v.y, c * v.x + d * v.y);
	}

	Matrix2 Matrix2::operator*(const Matrix2& other) const noexcept
	{
		Matrix2 newMatrix;
		Vector I = other.a * iHat;
		Vector II = other.c * jHat;
		Vector III = other.b * iHat;
		Vector IV = other.d * jHat;
		newMatrix.iHat = (I + II);
		newMatrix.jHat = (III + IV);
		return newMatrix;
	}

	bool Matrix2::operator==(const Matrix2& other) const noexcept
	{
		return iHat == other.iHat && jHat == other.jHat;
	}

	bool Matrix2::operator!=(const Matrix2& other) const noexcept
	{
		return jHat != other.iHat || jHat != other.jHat;
	}

	void Matrix2::Set(const f64& radians) noexcept
	{
		a = cos(radians);
		b = -sin(radians);
		c = sin(radians);
		d = cos(radians);
	}

	void Matrix2::Set(const f64& a, const f64& b, const f64& c, const f64& d) noexcept
	{
		this->a = a;
		this->b = b;
		this->c = c;
		this->d = d;
	}

	Matrix3::Matrix3() noexcept
	{
		a = 1;
		e = 1;
		i = 1;
	}

	Matrix3::Matrix3(const Matrix2& mat2) noexcept
	{
		a = mat2.a;
		b = mat2.b;
		d = mat2.c;
		e = mat2.d;
		c = 1, f = 1, g = 0, h = 0, i = 1;
	}

	Matrix3::Matrix3(const Matrix3& mat3) noexcept
	{
		iHat = mat3.iHat;
		jHat = mat3.jHat;
		kHat = mat3.kHat;
	}

	Matrix3::Matrix3(const f64& a, const f64& d, const f64& g, const f64& b, const f64& e, const f64& h, const f64& c, const f64& f, const f64& i) noexcept
	{
		this->a = a;
		this->d = d;
		this->g = g;
		this->b = b;
		this->e = e;
		this->h = h;
		this->c = c;
		this->f = f;
		this->i = i;
	}

	Matrix3::Matrix3(const Vector3& iHat, const Vector3& jHat, const Vector3& kHat) noexcept
	{
		this->iHat = iHat;
		this->jHat = jHat;
		this->kHat = kHat;
	}

	Matrix3& Matrix3::operator=(const Matrix3& other) noexcept
	{
		this->iHat = other.iHat;
		this->jHat = other.jHat;
		this->kHat = other.kHat;
		return *this;
	}

	Vector3 Matrix3::AxisX() const noexcept
	{
		return iHat;
	}

	Vector3 Matrix3::AxisY() const noexcept
	{
		return jHat;
	}

	Vector3 Matrix3::AxisZ() const noexcept
	{
		return kHat;
	}

	f64 Matrix3::Determinant() const noexcept
	{
		return Matrix2(e, f, h, i).Determinant() - 
			Matrix2(d, f, g, i).Determinant() -
			Matrix2(d, e, g, h).Determinant();
	}

	Matrix3 Matrix3::Transpose() const noexcept
	{
		return Matrix3(a, b, c, d, e, f, g, h, i);	
	}

	bool Matrix3::operator==(const Matrix3& other) const noexcept
	{
		return iHat == other.iHat && jHat == other.jHat && kHat == other.kHat;
	}

	bool Matrix3::operator!=(const Matrix3& other) const noexcept
	{
		return iHat != other.iHat || jHat != other.jHat || kHat != other.kHat;
	}

	Vector3 Matrix3::operator*(const Vector3& other) const noexcept
	{
		return Vector3(
			a * other.x + b * other.y + c * other.z,
			d * other.x + e * other.y + f * other.z,
			g * other.x + h * other.y + i * other.z
		);
	}

	Matrix3 Matrix3::operator*(const Matrix3& other) const noexcept
	{
		Matrix3 newMatrix;
		Vector3 vecs[9];
		vecs[0] = iHat * other.a;
		vecs[1] = jHat * other.b;
		vecs[2] = kHat * other.c;
		vecs[3] = iHat * other.d;
		vecs[4] = jHat * other.e;
		vecs[5] = kHat * other.f;
		vecs[6] = iHat * other.g;
		vecs[7] = jHat * other.h;
		vecs[8] = kHat * other.i;
		newMatrix.iHat = vecs[0] + vecs[3] + vecs[6];
		newMatrix.jHat = vecs[1] + vecs[4] + vecs[7];
		newMatrix.kHat = vecs[2] + vecs[5] + vecs[8];
		return newMatrix;
	}
}