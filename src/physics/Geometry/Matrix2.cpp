#include "physics/Geometry/Matrix.hpp"
namespace physics
{
    Matrix2::Matrix2() noexcept
	{
		arr[0] = 1;
		arr[1] = 0;
		arr[2] = 0;
		arr[3] = 1;
	}

	Matrix2::Matrix2(f64 radians) noexcept
	{
		arr[0] = cos(radians);
		arr[1] = -sin(radians);
		arr[2] = -arr[1];
		arr[3] = arr[0];
	}

	Matrix2::Matrix2(f64 a, f64 b, f64 c, f64 d) noexcept
	{
        arr[0] = a;
        arr[1] = b;
        arr[2] = c;
        arr[3] = d;
	}

	/*f64& Matrix2::operator()(size_t i, size_t j)
	{
		assert(i < 2 && j < 2);
		return arr[i * 2 + j];
	}

	const f64& Matrix2::operator()(size_t i, size_t j) const
	{
		assert(i < 2 && j < 2);
		return arr[i * 2 + j];
	}*/

	Matrix2& Matrix2::operator=(const Matrix2& other) noexcept
	{
        std::memcpy(arr, other.arr, sizeof(f64) * 4);
		return *this;
	}

	f64 Matrix2::Angle() const noexcept
	{
		return atan2(arr[2], arr[3]);
	}

	f64 Matrix2::GetDeterminant() const noexcept
	{
		return arr[0] * arr[3] - arr[1] * arr[2];
	}

	Matrix2 Matrix2::GetInverse() const noexcept
	{
		f64 invDet = 1.0 / GetDeterminant();
		Matrix2 m;
		m(0, 0) = arr[3] * invDet;
		m(0, 1) = -arr[1] * invDet;
		m(1, 0) = -arr[2] * invDet;
		m(1, 1) = arr[0] * invDet;
		return m;
	}

	Matrix2 Matrix2::GetTranspose() const noexcept
	{
		return Matrix2(arr[0], arr[2], arr[1], arr[3]);
	}

	/*Vector2 Matrix2::operator*(const Vector2& v) const noexcept
	{
		return Vector2(arr[0] * v.x + arr[1] * v.y, arr[2] * v.x + arr[3] * v.y);
	}*/

	/*Matrix2 Matrix2::operator*(const Matrix2& other) const noexcept
	{
		Matrix2 c;
        const Matrix2& a = *this;
        const Matrix2& b = other;
        c(0, 0) = a(0, 0) * b(0, 0) + a(0, 1) * b(1, 0);
        c(0, 1) = a(0, 0) * b(0, 1) + a(0, 1) * b(1, 1);
        c(1, 0) = a(1, 0) * b(0, 0) + a(1, 1) * b(1, 0);
        c(1, 1) = a(1, 0) * b(0, 1) + a(1, 1) * b(1, 1);
		return c;
	}*/

	Matrix2& Matrix2::operator*=(const Matrix2& other) noexcept
	{
		*this = *this * other;
		return *this;
	}

	bool Matrix2::operator==(const Matrix2& other) const noexcept
	{
		return arr[0] == other(0, 0) && arr[1] == other(0, 1) && arr[2] == other(1, 0) && arr[3] == other(1, 1);
	}

	bool Matrix2::operator!=(const Matrix2& other) const noexcept
	{
		return arr[0] != other(0, 0) || arr[1] != other(0, 1) || arr[2] != other(1, 0) || arr[3] != other(1, 1);
	}

	void Matrix2::Set(const f64& radians) noexcept
	{
		arr[0] = cos(radians);
		arr[1] = -sin(radians);
		arr[2] = -arr[1];
		arr[3] = arr[0];
	}

	void Matrix2::Set(const f64& a, const f64& b, const f64& c, const f64& d) noexcept
	{
        arr[0] = a;
        arr[1] = b;
        arr[2] = c;
        arr[3] = d;
	}
}