#include "geometry/main.hpp"
#ifndef PHYSICS_RK4_TMAX
#define PHYSICS_RK4_TMAX 50
#endif

namespace physics
{
	struct Transform;
	struct Transform
	{
		private:
			geo::Vector2 _position = geo::Vector2(0, 0);
			geo::Vector2 _centerOfMass = geo::Vector2(0, 0);
			geo::Matrix2 _scale = geo::Matrix2();
			geo::Matrix2 _rotation = geo::Matrix2();
			geo::Matrix3 _transformationMatrix = geo::Matrix3();
			geo::Matrix2 _rotxScale = geo::Matrix2();
		public:
			Transform() noexcept;
			Transform(const Transform& transform) noexcept;
			Transform(const geo::Vector2& position, const geo::Vector2& com, const geo::Vector2& scale, const geo::Matrix2& rotation) noexcept;
			Transform(const geo::Vector2& position, const geo::Vector2& com, const geo::Vector2& scale, f64 rotation) noexcept;
			Transform& operator=(const Transform& transform) noexcept;
			Transform& operator=(Transform&& transform) noexcept;
			virtual ~Transform() noexcept;
			bool operator==(const Transform& other) const noexcept;
			bool operator!=(const Transform& other) const noexcept;
			Transform operator*(const Transform& other) const noexcept;
			f64 GetAngle() const noexcept;
			geo::Vector2 GetCOM() const noexcept;
			Transform GetInverseTransform() const noexcept;
			geo::Vector2 GetPosition() const noexcept;
			geo::Matrix2 GetRotation() const noexcept;
			geo::Vector2 GetScale() const noexcept;
			geo::Matrix3 GetTransformationMatrix() const noexcept;
			void Rotate(f64 theta) noexcept;
			void SetAngle(f64 theta) noexcept;
			void SetCOM(const geo::Vector2& com) noexcept;
			void SetCOM(f64 x, f64 y) noexcept;
			void SetPosition(const geo::Vector2& position) noexcept;
			void SetPosition(f64 x, f64 y) noexcept;
			void SetRotation(const geo::Matrix2& rotation) noexcept;
			void SetScale(const geo::Vector2& scale) noexcept;
			void SetScale(f64 xScale, f64 yScale) noexcept;
			void Scale(f64 xScale, f64 yScale) noexcept;
			void Translate(geo::Vector2 offset) noexcept;
			geo::Vector2 TransformVector(const geo::Vector2& v) const noexcept;
			bool IsUnitTransform() const noexcept;
	};
}