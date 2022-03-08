#pragma once
#include <tuple>
#include "Vector.hpp"
typedef double f64;
typedef float f32;
typedef unsigned char uchar;
typedef unsigned int uint;
typedef unsigned long ulong;
typedef unsigned long long ulonglong;

namespace geometry
{

	// \brief A Line class containing two Vectors.
	class Line
	{
		private:
			f64 _length = 0;
			f64 _angle = 0;
		public:
			Line() noexcept;
			Line(const Vector& a, const Vector& b) noexcept;
			Line(const Line& l) noexcept;
			~Line() noexcept;
			Vector a;
			Vector b;
			bool operator==(const Line& l) const noexcept;
			bool operator!=(const Line& l) const noexcept;
			// \brief Returns the angle of the line.
			f64 angle() const noexcept;
			// \brief Returns a vector along the line based on the distance given.
			Vector GetVectorAlongLine(const f64& distance, const bool& startFromA = true) const noexcept;
			// \brief Returns a perpendicular line relative to 'this'.
			Line GetPerpendicular() const noexcept;
			// \brief Returns the length of the line.
			f64 length() const noexcept;
			// \brief Moves both a & b Vectors by an offset.
			void Move(const f64& offsetX, const f64& offsetY) noexcept;
			// \brief Returns a string form of the line.
			std::string ToString() const noexcept;
			// \brief Returns a tuple form of the line.
			std::tuple<std::tuple<f64, f64>, std::tuple<f64, f64>> ToTuple() const noexcept;
			// \brief Rotates both points of the line about a pivot point.
			void Rotate(const Vector& pivot, const f64& angle) noexcept;
			// \brief Checks if the given Vector is one the line.
			bool VectorIsOnLine(const Vector& v) const noexcept;
			// \brief Updates the angle and length values, if a & b Vectors are changed.
			void Update() noexcept;
	};
}