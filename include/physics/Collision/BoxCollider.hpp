#pragma once
#include "Collider.hpp"

namespace physics
{
	
	struct BoxCollider : public Collider
	{
		public:
			geo::Vector2 pos;
			geo::Vector2 dimensions;
			double& x = pos.x;
			double& y = pos.y;
			double& width = dimensions.x;
			double& height = dimensions.y;
			BoxCollider() noexcept;
			BoxCollider(const f64& width, const f64& height) noexcept;
			BoxCollider(const geo::Vector2& pos, const geo::Vector2& dimensions) noexcept;
			BoxCollider(const BoxCollider& b) noexcept;
			~BoxCollider() noexcept;
			BoxCollider& operator=(const BoxCollider& b);
			bool operator==(const Collider& b) const noexcept override;
			bool operator!=(const Collider& b) const noexcept override;
			virtual BoxCollider BoundingBox(const Transform& t = Transform()) const noexcept override;
			Collider* Clone() const noexcept override;
			virtual geo::Vector2 GetCenter() const noexcept override;
			virtual std::vector<geo::Vector2> GetPoints(const Transform& t = Transform()) const noexcept override;
			geo::Vector2 Max() const noexcept override;
			geo::Vector2 Min() const noexcept override;
			bool Overlaps(const BoxCollider& b) const noexcept;
			sf::RectangleShape ToShape() const noexcept;
			virtual CollisionPoints TestCollision(
				const Transform& transform,
				const Collider* collider,
				const Transform& colliderTransform) const noexcept override;
			virtual CollisionPoints TestCollision(
				const Transform& transform,
				const CircleCollider* collider,
				const Transform& colliderTransform) const noexcept override;
			virtual CollisionPoints TestCollision(
				const Transform& transform,
				const PolygonCollider* collider,
				const Transform& colliderTransform) const noexcept override;
			virtual CollisionPoints TestCollision(
				const Transform& transform,
				const BoxCollider* collider,
				const Transform& colliderTransform) const noexcept override;
			virtual CollisionPoints TestCollision(
				const Transform& transform,
				const MeshCollider* collider,
				const Transform& colliderTransform) const noexcept override;
	};
}