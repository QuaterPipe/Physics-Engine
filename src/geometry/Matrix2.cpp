#include "../include/geometry/Matrix.hpp"
namespace geo
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

	Matrix2::Row Matrix2::operator[](size_t index)
	{
		assert(index <= 1);
		switch (index)
		{
			case 0:
				return Matrix2::Row(arr);
			case 1:
				return Matrix2::Row(arr + 2);
		}
	}

	Matrix2::ConstRow Matrix2::operator[](size_t index) const
	{
		assert(index <= 1);
		switch (index)
		{
			case 0:
				return Matrix2::ConstRow(arr);
			case 1:
				return Matrix2::ConstRow(arr + 2);
		}
	}

	Matrix2& Matrix2::operator=(const Matrix2& other) noexcept
	{
        std::memcpy(arr, other.arr, sizeof(f64) * 4);
		return *this;
	}

	f64 Matrix2::Angle() const noexcept
	{
		return asin(arr[2]);
	}

	f64 Matrix2::Determinant() const noexcept
	{
		return arr[0] * arr[3] - arr[1] * arr[2];
	}

	Matrix2 Matrix2::Transpose() const noexcept
	{
		return Matrix2(arr[0], arr[2], arr[1], arr[3]);
	}

	Vector2 Matrix2::operator*(const Vector2& v) const noexcept
	{
		return Vector2(arr[0] * v.x + arr[1] * v.y, arr[2] * v.x + arr[3] * v.y);
	}

	Matrix2 Matrix2::operator*(const Matrix2& other) const noexcept
	{
		Matrix2 c;
        const Matrix2& a = *this;
        const Matrix2& b = other;
        c[0][0] = a[0][0] * b[0][0] + a[0][1] * b[1][0];
        c[0][1] = a[0][0] * b[0][1] + a[0][1] * b[1][1];
        c[1][0] = a[1][0] * b[0][0] + a[1][1] * b[1][0];
        c[1][1] = a[1][0] * b[0][1] + a[1][1] * b[1][1];
		return c;
	}

	Matrix2& Matrix2::operator*=(const Matrix2& other) noexcept
	{
		*this = *this * other;
		return *this;
	}

	bool Matrix2::operator==(const Matrix2& other) const noexcept
	{
		return std::equal(arr, arr + 4, other.arr);
	}

	bool Matrix2::operator!=(const Matrix2& other) const noexcept
	{
		return !std::equal(arr, arr + 4, other.arr);
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

    Matrix2::Row::Row(f64* ptr) noexcept
    : ptr(ptr)
    {
    }

    f64& Matrix2::Row::operator[](size_t index) const
    {
        assert(index <= 1);
		return ptr[index];
    }
    
    Matrix2::Row& Matrix2::Row::operator=(const Row& row)
    {
        ptr[0] = row[0];
        ptr[1] = row[1];
		return *this;
    }
	
	Matrix2::Row& Matrix2::Row::operator=(const ConstRow& row)
    {
        ptr[0] = row[0];
        ptr[1] = row[1];
		return *this;
    }
	
    Matrix2::ConstRow::ConstRow(const f64* ptr) noexcept
    : ptr(ptr)
    {
    }

    const f64& Matrix2::ConstRow::operator[](size_t index) const
    {
        assert(index <= 1);
		return ptr[index];
    }
}