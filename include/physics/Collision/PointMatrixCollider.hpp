#pragma once
#include "BoxCollider.hpp"

namespace physics
{
	struct PointMatrixCollider : public Collider
	{
		private:
			Vector2* _points = nullptr;
			size_t _pointCount;
			BoxCollider _boundingBox;
			Vector2 _min = Vector2::Infinity;
			Vector2 _max = -Vector2::Infinity;
			f64 _height;
			f64 _width;
		public:
			PointMatrixCollider() noexcept;
			PointMatrixCollider(size_t width, size_t height, size_t spacing) noexcept;
			PointMatrixCollider(const PointMatrixCollider& p) noexcept;
			PointMatrixCollider(PointMatrixCollider&& p) noexcept;
			~PointMatrixCollider() noexcept;
			BoxCollider BoundingBox(const Transform& t = Transform()) const noexcept override;
			Collider* Clone() const noexcept override;
			bool Contains(const Vector2& point, const Transform& t = Transform()) const noexcept override;
			f64 CrossSectionalArea(const Vector2& direction, const Transform& t = Transform()) const noexcept override;
			Vector2 GetCenter() const noexcept override;
			bool operator==(const Collider& c) const noexcept override;
			bool operator!=(const Collider& c) const noexcept override;
			Vector2 Max() const noexcept override;
			Vector2 Min() const noexcept override;
			f64 GetHeight() const noexcept;
			f64 GetWidth() const noexcept;
			const Vector2* GetPointArray() const noexcept;
			size_t GetPointCount() const noexcept;
			Vector2 GetPoint(size_t index) const noexcept;
			std::vector<Vector2> GetPoints(const Transform& t = Transform()) const noexcept override;
			void Release() noexcept;
			void SetPoint(size_t index, Vector2 newPosition) noexcept;
			virtual Manifold TestCollision(
				const Transform& transform,
				const Collider* collider,
				const Transform& colliderTransform) const noexcept override;
			virtual Manifold TestCollision(
				const Transform& transform,
				const CircleCollider* collider,
				const Transform& colliderTransform) const noexcept override;
			virtual Manifold TestCollision(
				const Transform& transform,
				const PolygonCollider* collider,
				const Transform& colliderTransform) const noexcept override;
			virtual Manifold TestCollision(
				const Transform& transform,
				const BoxCollider* collider,
				const Transform& colliderTransform) const noexcept override;
			virtual Manifold TestCollision(
				const Transform& transform,
				const MeshCollider* collider,
				const Transform& colliderTransform) const noexcept override;
			virtual Manifold TestCollision(
				const Transform& transform,
				const PointMatrixCollider* collider,
				const Transform& colliderTransform) const noexcept override;
			static size_t MAX_HEIGHT;
			static size_t MAX_WIDTH;
	};
}