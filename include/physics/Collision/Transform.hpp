#include "physics/Geometry/main.hpp"
#ifndef PHYSICS_RK4_TMAX
#define PHYSICS_RK4_TMAX 50
#endif

namespace physics
{
	struct Transform;
	struct Transform
	{
		private:
			Vector2 _position = Vector2(0, 0);
			Vector2 _centerOfMass = Vector2(0, 0);
			Matrix2 _scale = Matrix2();
			Matrix2 _rotation = Matrix2();
			Matrix3 _transformationMatrix = Matrix3();
			Matrix2 _rotxScale = Matrix2();
		public:
			Transform() noexcept;
			Transform(const Transform& transform) noexcept;
			Transform(const Vector2& position, const Vector2& com, const Vector2& scale, const Matrix2& rotation) noexcept;
			Transform(const Vector2& position, const Vector2& com, const Vector2& scale, f64 rotation) noexcept;
			Transform& operator=(const Transform& transform) noexcept;
			Transform& operator=(Transform&& transform) noexcept;
			virtual ~Transform() noexcept;
			bool operator==(const Transform& other) const noexcept;
			bool operator!=(const Transform& other) const noexcept;
			Transform operator*(const Transform& other) const noexcept;
			f64 GetAngle() const noexcept;
			Vector2 GetCOM() const noexcept;
			Transform GetInverseTransform() const noexcept;
			Vector2 GetPosition() const noexcept;
			Matrix2 GetRotation() const noexcept;
			inline Vector2 GetScale() const noexcept
			{
				return Vector2(_scale(0, 0), _scale(1, 1));
			}
			//Vector2 GetScale() const noexcept;
			Matrix3 GetTransformationMatrix() const noexcept;
			void Rotate(f64 theta) noexcept;
			void SetAngle(f64 theta) noexcept;
			void SetCOM(const Vector2& com) noexcept;
			void SetCOM(f64 x, f64 y) noexcept;
			void SetPosition(const Vector2& position) noexcept;
			void SetPosition(f64 x, f64 y) noexcept;
			void SetRotation(const Matrix2& rotation) noexcept;
			void SetScale(const Vector2& scale) noexcept;
			void SetScale(f64 xScale, f64 yScale) noexcept;
			void Scale(f64 xScale, f64 yScale) noexcept;
			void Translate(Vector2 offset) noexcept;
			//Vector2 TransformVector(const Vector2& v) const noexcept;
			inline Vector2 TransformVector(const Vector2& v) const noexcept
			{
				Vector2 result = _rotxScale * v;
				result.x += _transformationMatrix(0, 2);
				result.y += _transformationMatrix(1, 2);
				return result;
			}
			bool IsUnitTransform() const noexcept;
	};
}