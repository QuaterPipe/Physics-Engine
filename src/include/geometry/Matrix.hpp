#pragma once
#include "Math.hpp"
namespace geo
{
	/// \brief A 2x2 Matrix struct
	struct Matrix2
	{
		Vector2 iHat;
		Vector2 jHat;
		f64& a = iHat.x;
		f64& c = iHat.y;
		f64& b = jHat.x;
		f64& d = jHat.y;
		Matrix2() noexcept;
		Matrix2(const f64& radians) noexcept;
		Matrix2(const f64& a, const f64& b, const f64& c, const f64& d) noexcept;
		Matrix2(const Vector2& iHat, const Vector2& jHat) noexcept;
		Vector2 AxisX() const noexcept;
		Vector2 AxisY() const noexcept;
		f64 Angle() const noexcept;
		/// \brief Returns the determinant of the matrix.
		f64 Determinant() const noexcept;
		/// \brief Sets the matrix to rotational value.
		void Set(const f64& radians) noexcept;
		/// \brief Sets the matrix to given values.
		void Set(const f64& a, const f64& b, const f64& c, const f64& d) noexcept;
		/// \brief Flips the matrix over it's diagonal.
		Matrix2 Transpose() const noexcept;
		Matrix2& operator=(const Matrix2& other) noexcept;
		bool operator==(const Matrix2& other) const noexcept;
		bool operator!=(const Matrix2& other) const noexcept;
		Vector2 operator*(const Vector2& v) const noexcept;
		Matrix2 operator*(const Matrix2& m) const noexcept;
		void operator*=(const Matrix2& m) noexcept;
	};

	/// | a b c |
	/// | d e f |
	/// | g h i |
	struct Matrix3
	{
		Vector3 iHat; //a, d, g
		Vector3 jHat; //b, e, h
		Vector3 kHat; //c, f, i
		f64& a = iHat.x, d = iHat.y, g = iHat.z;
		f64& b = jHat.x, e = jHat.y, h = jHat.z;
		f64& c = kHat.x, f = kHat.y, i = kHat.z;
		Matrix3() noexcept;
		Matrix3(const Matrix2& mat2) noexcept;
		Matrix3(const Matrix3& mat3) noexcept;
		Matrix3(const f64& a, const f64& d, const f64& g, const f64& b, const f64& e, const f64& h, const f64& c, const f64& f, const f64& i) noexcept;
		Matrix3(const Vector3& iHat, const Vector3& jHat, const Vector3& kHat) noexcept;
		Matrix3& operator=(const Matrix3& other) noexcept;
		Vector3 AxisX() const noexcept;
		Vector3 AxisY() const noexcept;
		Vector3 AxisZ() const noexcept;
		f64 Determinant() const noexcept;
		Matrix3 Transpose() const noexcept;
		bool operator==(const Matrix3& other) const noexcept;
		bool operator!=(const Matrix3& other) const noexcept;
		Vector3 operator*(const Vector3& v) const noexcept;
		Matrix3 operator*(const Matrix3& other) const noexcept;
	};

	/*template <u32 N>
	struct Matrix
	{
		std::array<std::array<f64, N>, N> array;
		Matrix() noexcept
		{
		}
		Matrix(const Matrix<N>& mat) noexcept
		: array(mat.array)
		{
		}
		template <u32 Index>
		Vector<N> Axis() const
		{
			assert(Index < N && "Index out of range.");
			Vector<N> x;
			x.Set(array[Index]);
			return x;
		}
		std::array<f64, N>& operator[](size_t index) const
		{
			assert(index < N && "Index out of range.");
			return array[index];
		}
		bool operator==(const Matrix<N>& other) const noexcept
		{
			return array == other.array;
		}
		bool operator!=(const Matrix<N>& other) const noexcept
		{
			return array != other.array;
		}
		Vector<N> operator*(const Vector<N>& v) const noexcept
		{
			Vector<N> a;
			for (u32 i = 0; i < N; i++)
			{
				Vector<N> x;
				x.set(array[i]);
				a[i] = (x * v).Sum();
			}
			return a;
		}
		Matrix<N> operator*(const Matrix<N>& other) const noexcept
		{
			Matrix<N> x;
			for (u32 i = 0; i < N; i++)
			{
				for (u32 j = 0; j < N; j++)
				{
					for (u32 k = 0; k < N; k++)
						x[i][j] += array[i][k] * other.array[k][j];
				}
			}
			return x;
		}
	};*/
}