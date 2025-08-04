#pragma once
#include "BoxCollider.hpp"

namespace physics
{

	struct CircleCollider : public Collider
	{
		public:
			Vector2 center = Vector2();
			f64 radius = 0;
			CircleCollider() noexcept;
			CircleCollider(f64 radius) noexcept;
			CircleCollider(Vector2 center, f64 radius) noexcept;
			CircleCollider(const CircleCollider& c) noexcept;
			~CircleCollider() noexcept;
			bool operator==(const Collider& c) const noexcept override;
			bool operator!=(const Collider& c) const noexcept override;
			virtual BoxCollider BoundingBox(const Transform& t = Transform()) const noexcept override;
			virtual bool Contains(const Vector2& point, const Transform& t = Transform()) const noexcept override;
			Collider* Clone() const noexcept override;
			virtual Vector2 GetCenter() const noexcept override;
			Vector2 Max() const noexcept override;
			Vector2 Min() const noexcept override;
			virtual std::vector<Vector2> GetPoints(const Transform& t = Transform()) const noexcept override;
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