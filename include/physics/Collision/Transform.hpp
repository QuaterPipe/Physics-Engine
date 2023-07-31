#include "geometry/main.hpp"
#ifndef PHYSICS_RK4_TMAX
#define PHYSICS_RK4_TMAX 50
#endif

namespace physics
{
	struct Transform;
	struct Transform
	{
		public:
			geo::Vector2 position = geo::Vector2(0, 0);
			geo::Vector2 centerOfMass = geo::Vector2(0, 0);
			geo::Matrix2 scale = geo::Matrix2();
			geo::Matrix2 rotation = geo::Matrix2();
			Transform() noexcept;
			Transform(const Transform& transform) noexcept;
			Transform& operator=(const Transform& transform) noexcept;
			Transform& operator=(Transform&& transform) noexcept;
			virtual ~Transform() noexcept;
			bool operator==(const Transform& other) const noexcept;
			bool operator!=(const Transform& other) const noexcept;
			Transform operator*(const Transform& other) const noexcept;
			f64 GetAngle() const noexcept;
			geo::Vector2 GetScaleAsVector() const noexcept;
			geo::Matrix3 GetTransformationMatrix() const noexcept;
			void Rotate(f64 theta) noexcept;
			void SetAngle(f64 theta) noexcept;
			void Scale(f64 xScale, f64 yScale) noexcept;
			void Translate(geo::Vector2 offset) noexcept;
			geo::Vector2 TransformVector(const geo::Vector2& v) const noexcept;
			static void RK4Integrate(f64* position, f64* velocity, f64* acceleration, f64 dt);
			static void SymplecticEulerIntegrate(f64* position, f64* velocity, f64* acceleration, f64 dt);
	};
}