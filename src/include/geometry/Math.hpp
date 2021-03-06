#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <tuple>
#include <vector>
#include "Curve.hpp"
#include "Vector.hpp"
#include "Line.hpp"
typedef geo::Vector Vector2;
typedef double f64;
typedef float f32;
using u8 = u_int8_t;
using u16 = u_int16_t;
using u32 = u_int32_t;
using u64 = u_int64_t;
using i8 = int8_t;
using i16 = int16_t;
using i32 = int32_t;
using i64 = int64_t;

#define SQRD(X) \
( \
	(X) * (X)\
)
#define CUBD(X) \
( \
	(X) * (X) * (X) \
)
#define EPSILON 0.0000001
#define V_EPSILON geo::Vector(0.0000001, 0.0000001)
#define BIT(x) (1 << x)

namespace geo
{
	// \brief Compares two inputs, 1 if a is greater than b, 0 if they are equal,
	// and -1 if a is less than b.
	template <typename T>
	int Compare(T& a, T& b);
	/// \brief returns the calculated centroid of the given points.
	Vector Centroid(const Vector* start, const Vector* end) noexcept;
	// \brief Converts radians to degrees.
	f64 Degrees(const f64& radians) noexcept;
	// \brief Gets the distance between two Vectors.
	f64 Distance(const Vector& a, const Vector& b) noexcept;
	// \brief Gets the distance between two Vector3s.
	f64 Distance(const Vector3& a, const Vector3& b) noexcept;
	// \brief Gets the distance between two Vectors squared. It does not
	// use sqrt at all.
	f64 DistanceSquared(const Vector& a, const Vector& b) noexcept;
	// \brief Gets the distance between two Vector3s squared. It does not
	// use sqrt at all.
	f64 DistanceSquared(const Vector3& a, const Vector3& b) noexcept;
	// \brief Gets the minimum distance between a line and a Vector.
	f64 Distance(const Line& a, const Vector& b) noexcept;
	// \brief Fast way to get sqrt, not as accurate as sqrt().
	f64 FastSqrt(const f64& x) noexcept;
	// \brief Gets the angle between three Vectors.
	f64 GetAngle(const Vector& a, const Vector& b, const Vector& c) noexcept;
	// \brief Gets the angle from a slope.
	f64 GetAngle(const f64& slope) noexcept;
	// \brief Gets the angle between two Vectors.
	f64 GetAngle(const Vector& center, const Vector& Vector) noexcept;
	// \brief Returns a Vector on a circle based on the angle given.
	Vector GetVectorOnCircle(const Vector& center, const f64& radius, const f64& angle) noexcept;
	// \brief Returns the slope between two Vectors.
	f64 GetSlope(const Vector& a, const Vector& b) noexcept;
	// \brief Checks if two lines are intersecting are not.
	bool Intersecting(const Line& a, const Line& b, const bool& isInfLine = false) noexcept;
	// \brief Returns the point of Intersect between two lines if they are intersecting.
	Vector PointOfIntersect(const Line& la, const Line& lb, const bool& isInfLine = false) noexcept;
	//\brief
	// Quicksort algorithm that sorts using function to compare a to b.
	// The functions's first parameter is a, and it's second, b.
	// The function given must return negative numbers if a is less than b. 0 if they are equal,
	// and positive values a is greater than b. ex. returns: -1, 0, 1.
	template <typename T>
	void Sort(T* array, size_t low, size_t high, std::function<int(T&, T&)>& comparer = Compare) noexcept;
	/// \brief Returns the average of the given types.
	template <typename Numeric>
	Numeric Average(const Numeric* start, const Numeric* end) noexcept;
#include "Math.inl"
}