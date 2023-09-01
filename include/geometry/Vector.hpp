#pragma once
#include <array>
#include <cassert>
#define _USE_MATH_DEFINES
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <sstream>
#include <string>
#include <tuple>
#include <vector>
typedef double f64;
typedef float f32;
namespace geo
{
	class Line;
	struct Vector2
	{
		public:
			f64 x;
			f64 y;
			Vector2() noexcept;
			Vector2(const Vector2& v) noexcept;
			Vector2(f64 x, f64 y) noexcept;
			f64& operator[](size_t index);
			f64 operator[](size_t index) const;
			/// \brief returns the positive x & y values of *this vector
			Vector2 Abs() const noexcept;
			/// \brief Gets that angle between 'this' vector and the 'other' Vector.
			f64 Angle(const Vector2& other) const noexcept;
			/// \brief Returns the cross product between 'this' and the other Vector.
			f64 Cross(const Vector2& v) const noexcept;
			/// \brief Returns the cross product between a Vector and a double.
			static Vector2 Cross(const Vector2& v, f64 s) noexcept;
			/// \brief Returns the cross product between a double and a Vector.
			static Vector2 Cross(f64 s, const Vector2& v) noexcept;
			/// \brief Returns the dot product between 'this' and the 'other' Vector.
			f64 Dot(const Vector2& v) const noexcept;
			/// \brief Returns the magnitude of the Vector.
			f64 GetMagnitude() const noexcept;
			/// \brief Returns the squared magnitude of the Vector.
			f64 GetMagnitudeSquared() const noexcept;
			/// \brief Returns the magnitude of the Vector and using FastSqrt().
			f64 GetMagnitudeQuick() const noexcept;
			/// \brief Linearly interpolates a point with the other Vector.
			Vector2 Lerp(const Vector2& other, f64 t) const noexcept;
			/// \brief Offsets the Vector's x & y coordinates.
			void Move(f64 offsetX, f64 offsetY) noexcept;
			/// \brief Set's the Vector's magnitude to one.
			void Normalize() noexcept;
			/// \brief Returns a copy of the Vector that has been normalized.
			Vector2 Normalized() const noexcept;
			/// \brief Projects one Vector onto another.
			static Vector2 Projection(const Vector2& lhs, const Vector2& rhs) noexcept;
			/// \brief Projects a Vector onto a Line.
			static Vector2 Projection(const Vector2& vector, const Line& target) noexcept;
			/// \brief Return the quadrant of the given Vector relative to this Vector.
			int Quadrant(const Vector2& p) const noexcept;
			/// \brief Returns a vector reflected off of the given normal given
			Vector2 Reflection(const Vector2& normal) const noexcept;	
			/// \brief Reflects this Vector across the given normal
			void Reflect(const Vector2& normal) noexcept;
			/// \brief Rotates this Vector around the given point.
			void Rotate(const Vector2& p, f64 angle) noexcept;
			/// \brief Set's the Vector's x & y coordinates.
			void Set(f64 newX, f64 newY) noexcept;
			/// \brief Returns the Vector in string form.
			std::string ToString() const noexcept;
			/// \brief Returns the Vector in tuple form.
			std::tuple<f64, f64> ToTuple() const noexcept;
			Vector2 operator-() const noexcept;
			Vector2 operator+() const noexcept;
			bool operator==(const Vector2& v) const noexcept;
			bool operator!=(const Vector2& v) const noexcept;
			Vector2 operator-(const Vector2& v) const noexcept;
			Vector2 operator-(f64 d) const noexcept;
			void operator-=(const Vector2& v) noexcept;
			void operator-=(f64 d) noexcept;
			Vector2 operator+(const Vector2& v) const noexcept;
			Vector2 operator+(f64 d) const noexcept;
			void operator+=(const Vector2& v) noexcept;
			void operator+=(f64 d) noexcept;
			Vector2 operator/(const Vector2& v) const noexcept;
			Vector2 operator/(f64 d) const noexcept;
			void operator/=(const Vector2& v) noexcept;
			void operator/=(f64 d) noexcept;
			Vector2 operator*(const Vector2& v) const noexcept;
			Vector2 operator*(f64 d) const noexcept;
			void operator*=(const Vector2& v) noexcept;
			void operator*=(f64 d) noexcept;
			bool operator^(const Line& l) const noexcept;
			bool operator<(const Vector2& v) const noexcept;
			bool operator>(const Vector2& v) const noexcept;
			Vector2 operator()() const noexcept;
			static const Vector2 Origin;
			static const Vector2 Infinity;
			static const Vector2 iHat;
			static const Vector2 jHat;
	};
	Vector2 operator*(f64 d, const Vector2& v) noexcept;
	Vector2 operator+(f64 d, const Vector2& v) noexcept;
	Vector2 operator-(f64 d, const Vector2& v) noexcept;
	Vector2 operator/(f64 d, const Vector2& v) noexcept;

	std::ostream& operator<<(std::ostream& os, const Vector2& v);

	struct Vector3
	{
		public:
			f64 x;
			f64 y;
			f64 z;
			Vector3() noexcept;
			Vector3(const Vector3& v) noexcept;
			Vector3(f64 x, f64 y, f64 z) noexcept;
			f64& operator[](size_t index);
			f64 operator[](size_t index) const;
			/// \brief Returns the magnitude of the Vector3.
			f64 GetMagnitude() const noexcept;
			/// \brief Returns the magnitude of the Vector3 using FastSqrt().
			f64 GetMagnitudeQuick() const noexcept;
			/// \brief Returns the squared magnitude of the Vector3.
			f64 GetMagnitudeSquared() const noexcept;
			/// \brief Linearly interpolates a point with the other Vector3.
			Vector3 Lerp(const Vector3& other, f64 t) const noexcept;
			/// \brief Set's the Vector3's magnitude to one.
			void Normalize() noexcept;
			/// \brief Returns a normal version of the Vector3.
			Vector3 Normalized() const noexcept;
			/// \brief Returns the dot product between 'this' and the 'other' Vector3.
			f64 Dot(const Vector3& v) const noexcept;
			Vector3 operator-() const noexcept;
			Vector3 operator+() const noexcept;
			bool operator==(const Vector3& v) const noexcept;
			bool operator!=(const Vector3& v) const noexcept;
			Vector3 operator-(const Vector3& v) const noexcept;
			Vector3 operator-(f64 d) const noexcept;
			void operator-=(const Vector3& v) noexcept;
			void operator-=(f64 d) noexcept;
			Vector3 operator+(const Vector3& v) const noexcept;
			Vector3 operator+(f64 d) const noexcept;
			void operator+=(const Vector3& v) noexcept;
			void operator+=(f64 d) noexcept;
			Vector3 operator/(const Vector3& v) const noexcept;
			Vector3 operator/(f64 d) const noexcept;
			void operator/=(const Vector3& v) noexcept;
			void operator/=(f64 d) noexcept;
			Vector3 operator*(const Vector3& v) const noexcept;
			Vector3 operator*(f64 d) const noexcept;
			void operator*=(const Vector3& v) noexcept;
			void operator*=(f64 d) noexcept;
			bool operator<(const Vector3& v) const noexcept;
			bool operator>(const Vector3& v) const noexcept;
			Vector3 operator()() const noexcept;
			/// \brief Offsets the Vector3's x, y, and z coordinates.
			void Move(f64 offsetX, f64 offsetY, f64 offsetZ) noexcept;
			/// \brief Projects a Vector3 onto another Vector3.
			static Vector3 Projection(const Vector3& lhs, const Vector3& rhs) noexcept;
			/// \brief Returns a string version of Vector3.
			std::string ToString() const noexcept;
			/// \brief Returns a tuple version of Vector3.
			std::tuple<f64, f64, f64> ToTuple() const noexcept;
			/// \brief Set's the x, y, and z values of the Vector3 to the new given ones.
			void Set(f64 newX, f64 newY, f64 newZ) noexcept;
			static const Vector3 Origin;
			static const Vector3 Infinity;
			static const Vector3 iHat;
			static const Vector3 jHat;
			static const Vector3 kHat;
	};
	Vector3 operator*(f64 d, const Vector3& v) noexcept;
	Vector3 operator+(f64 d, const Vector3& v) noexcept;
	Vector3 operator-(f64 d, const Vector3& v) noexcept;
	Vector3 operator/(f64 d, const Vector3& v) noexcept;

	std::ostream& operator<<(std::ostream& os, const Vector3& v);

	struct Vector
	{
		public:
			Vector() noexcept;
			Vector(size_t size, f64 n = 0) noexcept;
			Vector(const std::vector<f64>& numbers) noexcept;
			Vector(const Vector& v) noexcept;
			std::vector<f64>::iterator begin() noexcept;
			std::vector<f64>::const_iterator begin() const noexcept;
			std::vector<f64>::iterator end() noexcept;
			std::vector<f64>::const_iterator end() const noexcept;
			size_t GetSize() const noexcept;
			f64 operator[](size_t i) const noexcept;
			f64& operator[](size_t i) noexcept;
			Vector Abs() const noexcept;
			/// \brief Returns the dot product between 'this' and the 'other' Vector.
			f64 Dot(const Vector& v) const;
			/// \brief Returns the magnitude of the Vector.
			f64 GetMagnitude() const noexcept;
			/// \brief Returns the squared magnitude of the Vector.
			f64 GetMagnitudeSquared() const noexcept;
			/// \brief Returns the magnitude of the Vector and using FastSqrt().
			f64 GetMagnitudeQuick() const noexcept;
			/// \brief Linearly interpolates a point with the other Vector.
			Vector Lerp(const Vector& other, f64 t) const;
			/// \brief Set's the Vector's magnitude to one.
			void Normalize() noexcept;
			/// \brief Returns a copy of the Vector that has been normalized.
			Vector Normalized() const noexcept;
			/// \brief Projects one Vector onto another.
			static Vector Projection(const Vector& lhs, const Vector& rhs);
			/// \brief Returns a Vector reflected off of the given normal given
			Vector Reflection(const Vector& normal) const;
			/// \brief Reflects this Vector across the given normal
			void Reflect(const Vector& normal);
			/// \brief Sets the vector to the given values
			void Set(const std::vector<f64>& values);
			/// \brief Sets the size of the vector to the given number
			void SetSize(size_t n);
			/// \brief Returns the sum of all values in the Vector
			f64 Sum() const noexcept;
			/// \brief Returns the Vector in string form.
			std::string ToString() const noexcept;
			Vector operator-() const noexcept;
			Vector operator+() const noexcept;
			bool operator==(const Vector& v) const;
			bool operator!=(const Vector& v) const;
			Vector operator-(const Vector& v) const;
			Vector operator-(f64 d) const noexcept;
			void operator-=(const Vector& v);
			void operator-=(f64 d) noexcept;
			Vector operator+(const Vector& v) const;
			Vector operator+(f64 d) const noexcept;
			void operator+=(const Vector& v);
			void operator+=(f64 d) noexcept;
			Vector operator/(const Vector& v) const;
			Vector operator/(f64 d) const noexcept;
			void operator/=(const Vector& v);
			void operator/=(f64 d) noexcept;
			Vector operator*(const Vector& v) const;
			Vector operator*(f64 d) const noexcept;
			void operator*=(const Vector& v);
			void operator*=(f64 d) noexcept;
			bool operator<(const Vector& v) const;
			bool operator>(const Vector& v) const;
			Vector operator()() const noexcept;
		private:
			size_t m_size;
			std::vector<f64> m_nums;
	};

	Vector operator*(f64 d, const Vector& v);
	Vector operator+(f64 d, const Vector& v);
	Vector operator-(f64 d, const Vector& v);
	Vector operator/(f64 d, const Vector& v);
	std::ostream& operator<<(std::ostream& os, const Vector& v) noexcept;
}