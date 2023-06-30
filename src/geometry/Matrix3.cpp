#include "geometry/Matrix.hpp"
namespace geo
{
    Matrix3::Matrix3() noexcept
	{
        arr[0] = 1;
		arr[1] = 0;
		arr[2] = 0;
		arr[3] = 0;
        arr[4] = 1;
        arr[5] = 0;
        arr[6] = 0;
        arr[7] = 0;
        arr[8] = 1;
	}

	Matrix3::Matrix3(const Matrix2& mat2) noexcept
	{
		arr[0] = mat2[0][0];
		arr[1] = mat2[0][1];
		arr[2] = 0;
		arr[3] = mat2[1][0];
		arr[4] = mat2[1][1];
		arr[5] = 0;
		arr[6] = 0;
		arr[7] = 0;
		arr[8] = 1;
	}

	Matrix3::Matrix3(const Matrix3& mat3) noexcept
	{
		std::memcpy(arr, mat3.arr, sizeof(f64) * 9);
	}

	Matrix3::Matrix3(f64 a, f64 b, f64 c, f64 d, f64 e, f64 f, f64 g, f64 h, f64 i) noexcept
	{
		arr[0] = a;
		arr[1] = b;
		arr[2] = c;
		arr[3] = d;
		arr[4] = e;
		arr[5] = f;
		arr[6] = g;
		arr[7] = h;
		arr[8] = i;
	}

	Matrix3& Matrix3::operator=(const Matrix3& other) noexcept
	{
		std::memcpy(arr, other.arr, sizeof(f64) * 9);
		return *this;
	}
	
	Matrix3::Row Matrix3::operator[](size_t index)
	{
		assert(index <= 3);
		if (!index)
			return Row(arr);
		else if (index == 1)
			return Row(arr + 3);
		else
			return Row(arr + 6);
	}
	
	Matrix3::ConstRow Matrix3::operator[](size_t index) const
	{
		assert(index <= 3);
		
		if (!index)
			return ConstRow(arr);
		else if (index == 1)
			return ConstRow(arr + 3);
		else
			return ConstRow(arr + 6);
	}

	Vector3 Matrix3::AxisX() const noexcept
	{
		return Vector3(arr[0], arr[3], arr[6]);
	}

	Vector3 Matrix3::AxisY() const noexcept
	{
		return Vector3(arr[1], arr[4], arr[7]);
	}

	Vector3 Matrix3::AxisZ() const noexcept
	{
		return Vector3(arr[2], arr[5], arr[8]);
	}
	
	f64 Matrix3::Determinant() const noexcept
	{
		return Matrix2(arr[4], arr[5], arr[7], arr[8]).Determinant() - 
			Matrix2(arr[3], arr[5], arr[6], arr[8]).Determinant() -
			Matrix2(arr[3], arr[4], arr[6], arr[7]).Determinant();
	}

	Matrix3 Matrix3::Transpose() const noexcept
	{
		return Matrix3(arr[0], arr[3], arr[6], arr[1], arr[4], arr[7], arr[2], arr[5], arr[8]);	
	}

	bool Matrix3::operator==(const Matrix3& other) const noexcept
	{
		return std::equal(arr, arr + 9, other.arr);
	}

	bool Matrix3::operator!=(const Matrix3& other) const noexcept
	{
		return !std::equal(arr, arr + 9, other.arr);
	}

	Vector3 Matrix3::operator*(const Vector3& other) const noexcept
	{
		return Vector3(
			arr[0] * other.x + arr[1] * other.y + arr[2] * other.z,
			arr[3] * other.x + arr[4] * other.y + arr[5] * other.z,
			arr[6] * other.x + arr[7] * other.y + arr[8] * other.z
		);
	}

	Matrix3 Matrix3::operator*(const Matrix3& other) const noexcept
	{
		Matrix3 c;
		const Matrix3& a = *this;
		const Matrix3& b = other;
		double m[24];
		m[1 ]= (a[0][0] + a[0][1] + a[0][2] - a[1][0] - a[1][1] - a[2][1] - a[2][2]) * b[1][1];
		m[2 ]= (a[0][0] - a[1][0]) * (-b[0][1] + b[1][1]);
		m[3 ]= a[1][1] * (-b[0][0] + b[0][1] + b[1][0] - b[1][1] - b[1][2] - b[2][0] + b[2][2]);
		m[4 ]= (-a[0][0] + a[1][0] + a[1][1]) * (b[0][0] - b[0][1] + b[1][1]);
		m[5 ]= (a[1][0] + a[1][1]) * (-b[0][0] + b[0][1]);
		m[6 ]= a[0][0] * b[0][0];
		m[7 ]= (-a[0][0] + a[2][0] + a[2][1]) * (b[0][0] - b[0][2] + b[1][2]);
		m[8 ]= (-a[0][0] + a[2][0]) * (b[0][2] - b[1][2]);
		m[9 ]= (a[2][0] + a[2][1]) * (-b[0][0] + b[0][2]);
		m[10]= (a[0][0] + a[0][1] + a[0][2] - a[1][1] - a[1][2] - a[2][0] - a[2][1]) * b[1][2];
		m[11]= a[2][1] * (-b[0][0] + b[0][2] + b[1][0] - b[1][1] - b[1][2] - b[2][0] + b[2][1]);
		m[12]= (-a[0][2] + a[2][1] + a[2][2]) * (b[1][1] + b[2][0] - b[2][1]);
		m[13]= (a[0][2] - a[2][2]) * (b[1][1] - b[2][1]);
		m[14]= a[0][2] * b[2][0];
		m[15]= (a[2][1] + a[2][2]) * (-b[2][0] + b[2][1]);
		m[16]= (-a[0][2] + a[1][1] + a[1][2]) * (b[1][2] + b[2][0] - b[2][2]);
		m[17]= (a[0][2] - a[1][2]) * (b[1][2] - b[2][2]);
		m[18]= (a[1][1] + a[1][2]) * (-b[2][0] + b[2][2]);
		m[19]= a[0][1] * b[1][0];
		m[20]= a[1][2] * b[2][1];
		m[21]= a[1][0] * b[0][2];
		m[22]= a[2][0] * b[0][1];
		m[23]= a[2][2] * b[2][2];

		c[0][0] = m[6] + m[14] + m[19];
		c[0][1] = m[1] + m[4] + m[5] + m[6] + m[12] + m[14] + m[15];
		c[0][2] = m[6] + m[7] + m[9] + m[10] + m[14] + m[16] + m[18];
		c[1][0] = m[2] + m[3] + m[4] + m[6] + m[14] + m[16] + m[17];
		c[1][1] = m[2] + m[4] + m[5] + m[6] + m[20];
		c[1][2] = m[14] + m[16] + m[17] + m[18] + m[21];
		c[2][0] = m[6] + m[7] + m[8] + m[11] + m[12] + m[13] + m[14];
		c[2][1] = m[12] + m[13] + m[14] + m[15] + m[22];
		c[2][2] = m[6] + m[7] + m[8] + m[9] + m[23];
		return c;
	}

	void Matrix3::operator*=(const Matrix3& other) noexcept
	{
		*this = *this * other;
	}

	Matrix3::Row::Row(f64* ptr) noexcept
	: ptr(ptr)
	{
	}

	f64& Matrix3::Row::operator[](size_t index) const
	{
		assert(index <= 2);
		return ptr[index];
	}

	Matrix3::Row& Matrix3::Row::operator=(const Row& other)
	{
		ptr[0] = other[0];
		ptr[1] = other[1];
		ptr[2] = other[2];
		return *this;
	}

	Matrix3::Row& Matrix3::Row::operator=(const ConstRow& other)
	{
		ptr[0] = other[0];
		ptr[1] = other[1];
		ptr[2] = other[2];
		return *this;
	}

	Matrix3::ConstRow::ConstRow(const f64* ptr) noexcept
	: ptr(ptr)
	{
	}

	const f64& Matrix3::ConstRow::operator[](size_t index) const
	{
		assert(index <= 2);
		return ptr[index];
	}

}