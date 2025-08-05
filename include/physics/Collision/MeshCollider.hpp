#pragma once
#include "BoxCollider.hpp"

namespace physics
{
	//0x05
	struct MeshCollider : public Collider
	{
		public:
			std::vector<Collider*> colliders;
			MeshCollider() noexcept;
			MeshCollider(const std::vector<Collider*>& colliders) noexcept;
			MeshCollider(const MeshCollider& m) noexcept;
			~MeshCollider() noexcept;
			bool operator==(const Collider& c) const noexcept override;
			bool operator!=(const Collider& c) const noexcept override;
			virtual BoxCollider BoundingBox(const Transform& t = Transform()) const noexcept override;
			Collider* Clone() const noexcept override;
			virtual bool Contains(const Vector2& point, const Transform& t = Transform()) const noexcept override;
			virtual f64 CrossSectionalArea(const Vector2& direction) const noexcept override;
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