#pragma once
#include <cstring>
#include "Math.hpp"
namespace geo
{
	/// \brief A 2x2 Matrix struct
	struct Matrix2
	{
		struct ConstRow
		{
			ConstRow(const f64* ptr) noexcept;
			const f64& operator[](size_t index) const;
			private:
				const f64* ptr = nullptr;
		};

		struct Row
		{
			Row(f64* ptr) noexcept;
			f64& operator[](size_t index) const;
			Row& operator=(const Row& row);
			Row& operator=(const ConstRow& row);
			private:
				f64* ptr = nullptr;
		};

		f64 arr[4];
		Matrix2() noexcept;
		Matrix2(f64 radians) noexcept;
		Matrix2(f64 a, f64 b, f64 c, f64 d) noexcept;
		Row operator[](size_t index);
		ConstRow operator[](size_t index) const;
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
		Matrix2& operator*=(const Matrix2& other) noexcept;
	};

	struct Matrix3
	{
		struct ConstRow
		{
			ConstRow(const f64* ptr) noexcept;
			const f64& operator[](size_t index) const;
			private:
				const f64* ptr = NULL;
		};

		struct Row
		{
			Row(f64* ptr) noexcept;
			f64& operator[](size_t index) const;
			Row& operator=(const Row& row);
			Row& operator=(const ConstRow& row);
			private:
				f64* ptr = NULL;
		};
		f64 arr[9];
		Matrix3() noexcept;
		Matrix3(const Matrix2& mat2) noexcept;
		Matrix3(const Matrix3& mat3) noexcept;
		Matrix3(f64 a, f64 b, f64 c, f64 d, f64 e, f64 f, f64 g, f64 h, f64 i) noexcept;
		Row operator[](size_t index);
		ConstRow operator[](size_t index) const;
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
		Matrix3& operator*=(const Matrix3& other) noexcept;
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
			Matrix(size_t width, size_t height, f64 n = 0) noexcept;
			Matrix(const f64* arr, size_t width, size_t height) noexcept;
			Matrix& operator=(const Matrix& other) noexcept;
			Vector Axis(u32 index) const;
			const Row operator[](size_t index) const;
			Row operator[](size_t index);
			bool operator==(const Matrix& other) const noexcept;
			bool operator!=(const Matrix& other) const noexcept;
			Vector operator*(const Vector& v) const;
			Matrix operator*(const Matrix& other) const;
			Matrix& operator*=(const Matrix& other);
			i32 GetDeterminant() const;
			size_t GetHeight() const noexcept;
			size_t GetWidth() const noexcept;
			Matrix GetTranspose() const;
		private:
			size_t _width;
			size_t _height;
	};
}