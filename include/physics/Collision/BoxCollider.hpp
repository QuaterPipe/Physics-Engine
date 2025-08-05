#pragma once
#include "Collider.hpp"

namespace physics
{
	
	struct BoxCollider : public Collider
	{
		public:
			Vector2 pos;
			Vector2 dimensions;
			f64& x = pos.x;
			f64& y = pos.y;
			f64& width = dimensions.x;
			f64& height = dimensions.y;
			BoxCollider() noexcept;
			BoxCollider(const f64& width, const f64& height) noexcept;
			BoxCollider(const Vector2& pos, const Vector2& dimensions) noexcept;
			BoxCollider(const BoxCollider& b) noexcept;
			~BoxCollider() noexcept;
			BoxCollider& operator=(const BoxCollider& b);
			bool operator==(const Collider& b) const noexcept override;
			bool operator!=(const Collider& b) const noexcept override;
			virtual BoxCollider BoundingBox(const Transform& t = Transform()) const noexcept override;
			virtual bool Contains(const Vector2& point, const Transform& t = Transform()) const noexcept override;
			Collider* Clone() const noexcept override;
			virtual f64 CrossSectionalArea(const Vector2& direction) const noexcept override;
			virtual Vector2 GetCenter() const noexcept override;
			virtual std::vector<Vector2> GetPoints(const Transform& t = Transform()) const noexcept override;
			Vector2 Max() const noexcept override;
			Vector2 Min() const noexcept override;
			bool Overlaps(const BoxCollider& b) const noexcept;
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
	};
}