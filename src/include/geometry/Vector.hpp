#pragma once
#include <cmath>
#include <vector>
#include <tuple>
typedef double f64;
typedef float f32;
typedef unsigned char uchar;
typedef unsigned int uint;
typedef unsigned long ulong;
typedef unsigned long long ulonglong;
namespace geometry
{
	class Line;
	struct Vector
	{
		public:
			f64 x;
			f64 y;
			Vector() noexcept;
			Vector(const f64& x, const f64& y) noexcept;
			// \brief returns the positive x & y values of *this vector
			Vector Abs() const noexcept;
			// \brief Gets that angle between 'this' vector and the 'other' Vector.
			f64 Angle(const Vector& other) const noexcept;
			// \brief Returns the cross product between 'this' and the other Vector.
			f64 Cross(const Vector& v) const noexcept;
			// \brief Returns the cross product between a Vector and a double.
			static Vector Cross(const Vector& v, const f64& s) noexcept;
			// \brief Returns the cross product between a double and a Vector.
			static Vector Cross(const f64& s, const Vector& v) noexcept;
			// \brief Returns the dot product between 'this' and the 'other' Vector.
			f64 Dot(const Vector& v) const noexcept;
			// \brief Returns the magnitude of the Vector.
			f64 GetMagnitude() const noexcept;
			/// \brief Returns the squared magnitude of the Vector.
			f64 GetMagnitudeSquared() const noexcept;
			// \brief Returns the magnitude of the Vector and using FastSqrt().
			f64 GetMagnitudeQuick() const noexcept;
			// \brief Linearly interpolates a point with the other Vector.
			Vector Lerp(const Vector& other, const f64& t) const noexcept;
			// \brief Offsets the Vector's x & y coordinates.
			void Move(const f64& offsetX, const f64& offsetY) noexcept;
			// \brief Set's the Vector's magnitude to one.
			void Normalize() noexcept;
			// \brief Returns a copy of the Vector that has been normalized.
			Vector Normalized() const noexcept;
			// \brief Projects one Vector onto another.
			static Vector Projection(const Vector& lhs, const Vector& rhs) noexcept;
			// \brief Projects a Vector onto a Line.
			static Vector Projection(const Vector& vector, const Line& target) noexcept;
			// \brief Return the quadrant of the given Vector relative to this Vector.
			int Quadrant(const Vector& p) const noexcept;
			// \brief Returns a vector reflected off of the given normal given
			Vector Reflection(const Vector& normal) const noexcept;	
			// \brief Reflects this Vector across the given normal
			void Reflect(const Vector& normal) noexcept;
			// \brief Rotates this Vector around the given point.
			void Rotate(const Vector& p, const f64& angle) noexcept;
			// \brief Set's the Vector's x & y coordinates.
			void Set(const f64& newX, const f64& newY) noexcept;
			// \brief Returns the Vector in string form.
			std::string ToString() const noexcept;
			// \brief Returns the Vector in tuple form.
			std::tuple<f64, f64> ToTuple() const noexcept;
			Vector operator-() const noexcept;
			Vector operator+() const noexcept;
			bool operator==(const Vector& v) const noexcept;
			bool operator!=(const Vector& v) const noexcept;
			Vector operator-(const Vector& v) const noexcept;
			Vector operator-(const f64& d) const noexcept;
			void operator-=(const Vector& v) noexcept;
			void operator-=(const f64& d) noexcept;
			Vector operator+(const Vector& v) const noexcept;
			Vector operator+(const f64& d) const noexcept;
			void operator+=(const Vector& v) noexcept;
			void operator+=(const f64& d) noexcept;
			Vector operator/(const Vector& v) const noexcept;
			Vector operator/(const f64& d) const noexcept;
			void operator/=(const Vector& v) noexcept;
			void operator/=(const f64& d) noexcept;
			Vector operator*(const Vector& v) const noexcept;
			Vector operator*(const f64& d) const noexcept;
			void operator*=(const Vector& v) noexcept;
			void operator*=(const f64& d) noexcept;
			bool operator^(const Line& l) const noexcept;
			bool operator<(const Vector& v) const noexcept;
			bool operator>(const Vector& v) const noexcept;
			Vector operator()() const noexcept;
			static const Vector Origin;
			static const Vector Infinity;
			static const Vector iHat;
			static const Vector jHat;
	};
	Vector operator*(const f64& d, const Vector& v) noexcept;
	Vector operator+(const f64& d, const Vector& v) noexcept;

	struct Vector3
	{
		public:
			f64 x;
			f64 y;
			f64 z;
			Vector3() noexcept;
			Vector3(const f64& x, const f64& y, const f64& z) noexcept;
			// \brief Returns the magnitude of the Vector3.
			f64 GetMagnitude() const noexcept;
			// \brief Returns the magnitude of the Vector3 using FastSqrt().
			f64 GetMagnitudeQuick() const noexcept;
			// \brief Returns the squared magnitude of the Vector3.
			f64 GetMagnitudeSquared() const noexcept;
			// \brief Linearly interpolates a point with the other Vector3.
			Vector3 Lerp(const Vector3& other, const f64& t) const noexcept;
			// \brief Set's the Vector3's magnitude to one.
			void Normalize() noexcept;
			// \brief Returns a normal version of the Vector3.
			Vector3 Normalized() const noexcept;
			// \brief Returns the dot product between 'this' and the 'other' Vector3.
			f64 Dot(const Vector3& v) const noexcept;
			Vector3 operator-() const noexcept;
			Vector3 operator+() const noexcept;
			bool operator==(const Vector3& v) const noexcept;
			bool operator!=(const Vector3& v) const noexcept;
			Vector3 operator-(const Vector3& v) const noexcept;
			Vector3 operator-(const f64& d) const noexcept;
			void operator-=(const Vector3& v) noexcept;
			void operator-=(const f64& d) noexcept;
			Vector3 operator+(const Vector3& v) const noexcept;
			Vector3 operator+(const f64& d) const noexcept;
			void operator+=(const Vector3& v) noexcept;
			void operator+=(const f64& d) noexcept;
			Vector3 operator/(const Vector3& v) const noexcept;
			Vector3 operator/(const f64& d) const noexcept;
			void operator/=(const Vector3& v) noexcept;
			void operator/=(const f64& d) noexcept;
			Vector3 operator*(const Vector3& v) const noexcept;
			Vector3 operator*(const f64& d) const noexcept;
			void operator*=(const Vector3& v) noexcept;
			void operator*=(const f64& d) noexcept;
			bool operator<(const Vector3& v) const noexcept;
			bool operator>(const Vector3& v) const noexcept;
			Vector3 operator()() const noexcept;
			// \brief Offsets the Vector3's x, y, and z coordinates.
			void Move(const f64& offsetX, const f64& offsetY, const f64& offsetZ) noexcept;
			// \brief Projects a Vector3 onto another Vector3.
			static Vector3 Projection(const Vector3& lhs, const Vector3& rhs) noexcept;
			// \brief Returns a string version of Vector3.
			std::string ToString() const noexcept;
			// \brief Returns a tuple version of Vector3.
			std::tuple<f64, f64, f64> ToTuple() const noexcept;
			// \brief Set's the x, y, and z values of the Vector3 to the new given ones.
			void Set(const f64& newX, const f64& newY, const f64& newZ) noexcept;
			static const Vector3 Origin;
			static const Vector3 Infinity;
			static const Vector3 iHat;
			static const Vector3 jHat;
			static const Vector3 kHat;
	};
}
geometry::Vector cos(const geometry::Vector& v);
geometry::Vector sin(const geometry::Vector& v);
geometry::Vector tan(const geometry::Vector& v);