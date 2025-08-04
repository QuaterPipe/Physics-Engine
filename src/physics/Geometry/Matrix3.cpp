#include "physics/Geometry/Matrix.hpp"
namespace physics
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
		arr[0] = mat2(0, 0);
		arr[1] = mat2(0, 1);
		arr[2] = 0;
		arr[3] = mat2(1, 0);
		arr[4] = mat2(1, 1);
		arr[5] = 0;
		arr[6] = 0;
		arr[7] = 0;
		arr[8] = 1;
	}

	Matrix3::Matrix3(const Matrix3& mat3) noexcept
	{
		for (int i = 0; i < 9; i++)
			arr[i] = mat3.arr[i];
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
		for (int i = 0; i < 9; i++)
			arr[i] = other.arr[i];
		return *this;
	}

	f64& Matrix3::operator()(size_t i, size_t j)
	{
		assert(i < 3 && j < 3);
		return arr[i * 3 + j];
	}

	const f64& Matrix3::operator()(size_t i, size_t j) const
	{
		assert(i < 3 && j < 3);
		return arr[i * 3 + j];
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
	
	f64 Matrix3::GetDeterminant() const noexcept
	{
		return Matrix2(arr[4], arr[5], arr[7], arr[8]).GetDeterminant() - 
			Matrix2(arr[3], arr[5], arr[6], arr[8]).GetDeterminant() -
			Matrix2(arr[3], arr[4], arr[6], arr[7]).GetDeterminant();
	}

	Matrix3 Matrix3::GetInverse() const noexcept
	{
		f64 invDet = 1.0 / GetDeterminant();
		const Matrix3& m = *this;	 
		Matrix3 mInv;
		mInv(0, 0) = (m(1, 1) * m(2, 2) - m(2, 1) * m(1, 2)) * invDet;
		mInv(0, 1) = (m(0, 2) * m(2, 1) - m(0, 1) * m(2, 2)) * invDet;
		mInv(0, 2) = (m(0, 1) * m(1, 2) - m(0, 2) * m(1, 1)) * invDet;
		mInv(1, 0) = (m(1, 2) * m(2, 0) - m(1, 0) * m(2, 2)) * invDet;
		mInv(1, 1) = (m(0, 0) * m(2, 2) - m(0, 2) * m(2, 0)) * invDet;
		mInv(1, 2) = (m(1, 0) * m(0, 2) - m(0, 0) * m(1, 2)) * invDet;
		mInv(2, 0) = (m(1, 0) * m(2, 1) - m(2, 0) * m(1, 1)) * invDet;
		mInv(2, 1) = (m(2, 0) * m(0, 1) - m(0, 0) * m(2, 1)) * invDet;
		mInv(2, 2) = (m(0, 0) * m(1, 1) - m(1, 0) * m(0, 1)) * invDet;
		return mInv;
	}

	Matrix3 Matrix3::GetTranspose() const noexcept
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
		f64 m[24];
		m[1 ]= (a(0, 0) + a(0, 1) + a(0, 2) - a(1, 0) - a(1, 1) - a(2, 1) - a(2, 2)) * b(1, 1);
		m[2 ]= (a(0, 0) - a(1, 0)) * (-b(0, 1) + b(1, 1));
		m[3 ]= a(1, 1) * (-b(0, 0) + b(0, 1) + b(1, 0) - b(1, 1) - b(1, 2) - b(2, 0) + b(2, 2));
		m[4 ]= (-a(0, 0) + a(1, 0) + a(1, 1)) * (b(0, 0) - b(0, 1) + b(1, 1));
		m[5 ]= (a(1, 0) + a(1, 1)) * (-b(0, 0) + b(0, 1));
		m[6 ]= a(0, 0) * b(0, 0);
		m[7 ]= (-a(0, 0) + a(2, 0) + a(2, 1)) * (b(0, 0) - b(0, 2) + b(1, 2));
		m[8 ]= (-a(0, 0) + a(2, 0)) * (b(0, 2) - b(1, 2));
		m[9 ]= (a(2, 0) + a(2, 1)) * (-b(0, 0) + b(0, 2));
		m[10]= (a(0, 0) + a(0, 1) + a(0, 2) - a(1, 1) - a(1, 2) - a(2, 0) - a(2, 1)) * b(1, 2);
		m[11]= a(2, 1) * (-b(0, 0) + b(0, 2) + b(1, 0) - b(1, 1) - b(1, 2) - b(2, 0) + b(2, 1));
		m[12]= (-a(0, 2) + a(2, 1) + a(2, 2)) * (b(1, 1) + b(2, 0) - b(2, 1));
		m[13]= (a(0, 2) - a(2, 2)) * (b(1, 1) - b(2, 1));
		m[14]= a(0, 2) * b(2, 0);
		m[15]= (a(2, 1) + a(2, 2)) * (-b(2, 0) + b(2, 1));
		m[16]= (-a(0, 2) + a(1, 1) + a(1, 2)) * (b(1, 2) + b(2, 0) - b(2, 2));
		m[17]= (a(0, 2) - a(1, 2)) * (b(1, 2) - b(2, 2));
		m[18]= (a(1, 1) + a(1, 2)) * (-b(2, 0) + b(2, 2));
		m[19]= a(0, 1) * b(1, 0);
		m[20]= a(1, 2) * b(2, 1);
		m[21]= a(1, 0) * b(0, 2);
		m[22]= a(2, 0) * b(0, 1);
		m[23]= a(2, 2) * b(2, 2);

		c(0, 0) = m[6] + m[14] + m[19];
		c(0, 1) = m[1] + m[4] + m[5] + m[6] + m[12] + m[14] + m[15];
		c(0, 2) = m[6] + m[7] + m[9] + m[10] + m[14] + m[16] + m[18];
		c(1, 0) = m[2] + m[3] + m[4] + m[6] + m[14] + m[16] + m[17];
		c(1, 1) = m[2] + m[4] + m[5] + m[6] + m[20];
		c(1, 2) = m[14] + m[16] + m[17] + m[18] + m[21];
		c(2, 0) = m[6] + m[7] + m[8] + m[11] + m[12] + m[13] + m[14];
		c(2, 1) = m[12] + m[13] + m[14] + m[15] + m[22];
		c(2, 2) = m[6] + m[7] + m[8] + m[9] + m[23];
		return c;
	}

	Matrix3& Matrix3::operator*=(const Matrix3& other) noexcept
	{
		*this = *this * other;
		return *this;
	}
}