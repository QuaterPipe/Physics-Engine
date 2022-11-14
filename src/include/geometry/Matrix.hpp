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

	struct Matrix
	{
		public:
			struct Row
			{
				f64* data;
				size_t start;
				size_t end;
				Row(f64* data, size_t start, size_t end) noexcept;
				Row() = delete;
				f64& operator[](size_t index);
				const f64& operator[](size_t index) const;
				Row& operator=(const Row& other);
			};
			f64* array = NULL;
			~Matrix() noexcept;
			Matrix() noexcept;
			Matrix(const Matrix& mat) noexcept;
			Matrix(Matrix && mat) noexcept;
			Matrix(u32 width, u32 height, f64 n = 0) noexcept;
			Matrix(f64* arr, u32 width, u32 height) noexcept;
			Matrix& operator=(const Matrix& other) noexcept;
			Vector Axis(u32 index) const;
			const Row operator[](size_t index) const;
			Row operator[](size_t index);
			bool operator==(const Matrix& other) const noexcept;
			bool operator!=(const Matrix& other) const noexcept;
			Vector operator*(const Vector& v) const noexcept;
			Matrix operator*(const Matrix& other) const noexcept;
			i32 GetDeterminant() const;
			size_t GetHeight() const noexcept;
			size_t GetWidth() const noexcept;
			Matrix GetTranspose() const;
		private:
			size_t _width;
			size_t _height;
	};
}