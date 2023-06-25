#pragma once
#include <algorithm>
#include <cstdint>
#include <functional>
#include "Curve.hpp"
#include "Vector.hpp"
#include "Line.hpp"
typedef double f64;
typedef float f32;
using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;
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
#define EPSILON 10e-8
#define V3_EPSILON geo::Vector2(10e-8, 10e-8, 10e-8)
#define V2_EPSILON geo::Vector2(10e-8, 10e-8)
#define V_EPSILON(X) geo::Vector(X, 10e-8)
#define BIT(x) (1 << x)

namespace geo
{
	/// \brief returns the closest Line to the Vector2
	Line ClosestLine(std::vector<geo::Line> lines, Vector2 vector) noexcept;
	/// \brief Compares two inputs, 1 if a is greater than b, 0 if they are equal,
	/// and -1 if a is less than b.
	template <typename T>
	int Compare(T& a, T& b);
	/// \brief returns the calculated centroid of the given points.
	Vector2 Centroid(const Vector2* start, const Vector2* end) noexcept;
	/// \brief returns the calculated centroid of the given points.
	Vector2 Centroid(const std::vector<geo::Vector2>& vertexes) noexcept;
	/// \brief Converts radians to degrees.
	f64 Degrees(const f64& radians) noexcept;
	/// \brief Gets the distance between two Vectors.
	f64 Distance(const Vector2& a, const Vector2& b) noexcept;
	/// \brief Gets the distance between a Line and a Vector2.
	f64 Distance(const Line& a, const Vector2& b) noexcept;
	/// \brief Gets the distance between two Vector3s.
	f64 Distance(const Vector3& a, const Vector3& b) noexcept;
	/// \brief Gets the distance between two Vectors squared. It does not
	/// use sqrt at all.
	f64 DistanceSquared(const Vector2& a, const Vector2& b) noexcept;
	/// \brief Gets the distance between a Line and a Vector2 squared. It does not
	/// use sqrt at all.
	f64 DistanceSquared(const Line& a, const Vector2& b) noexcept;
	/// \brief Gets the distance between two Vector3s squared. It does not
	/// use sqrt at all.
	f64 DistanceSquared(const Vector3& a, const Vector3& b) noexcept;
	/// \brief Gets the minimum distance between a line and a Vector.
	f64 Distance(const Line& a, const Vector2& b) noexcept;
	/// \brief Fast way to get sqrt, not as accurate as sqrt().
	f64 FastSqrt(const f64& x) noexcept;
	/// \brief Gets the angle between three Vectors.
	f64 GetAngle(const Vector2& a, const Vector2& b, const Vector2& c) noexcept;
	/// \brief Gets the angle from a slope.
	f64 GetAngle(const f64& slope) noexcept;
	/// \brief Gets the angle between two Vectors.
	f64 GetAngle(const Vector2& center, const Vector2& Vector) noexcept;
	/// \brief Returns a Vector on a circle based on the angle given.
	Vector2 GetVectorOnCircle(const Vector2& center, const f64& radius, const f64& angle) noexcept;
	/// \brief Returns the slope between two Vectors.
	f64 GetSlope(const Vector2& a, const Vector2& b) noexcept;
	/// \brief Checks if two lines are intersecting are not.
	bool Intersecting(const Line& a, const Line& b, const bool& isInfLine = false) noexcept;
	/// \brief Returns the point of Intersect between two lines if they are intersecting.
	Vector2 PointOfIntersect(const Line& la, const Line& lb, const bool& isInfLine = false) noexcept;
	//\brief
	/// Quicksort algorithm that sorts using function to compare a to b.
	/// The functions's first parameter is a, and it's second, b.
	/// The function given must return negative numbers if a is less than b. 0 if they are equal,
	/// and positive values a is greater than b. ex. returns: -1, 0, 1.
	template <typename T>
	void Sort(T* array, size_t low, size_t high, std::function<int(T&, T&)>& comparer = Compare) noexcept;
	/// \brief Returns the average of the given types.
	template <typename Numeric>
	Numeric Average(const Numeric* start, const Numeric* end) noexcept;
#include "Math.inl"
}
