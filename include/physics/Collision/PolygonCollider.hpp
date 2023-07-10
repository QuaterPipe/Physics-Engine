#pragma once
#include "BoxCollider.hpp"


namespace physics
{
	
	//0x04
	struct PolygonCollider : public Collider
	{
		private:
			std::vector<geo::Vector2> _points;
			std::vector<geo::Vector2> _normals;
		public:
			geo::Vector2 pos;
			PolygonCollider();
			PolygonCollider(const BoxCollider& b) noexcept;
			PolygonCollider(const PolygonCollider& p) noexcept;
			PolygonCollider(const geo::Vector2& pos, f64 sideLength=1, unsigned long count=3) noexcept;
			PolygonCollider(const geo::Vector2& pos, const geo::Vector2& a, const geo::Vector2& b, const geo::Vector2& c, std::initializer_list<geo::Vector2> extra={}) noexcept;
			~PolygonCollider() noexcept;
			virtual BoxCollider BoundingBox(const Transform& t = Transform()) const noexcept override;
			virtual bool Contains(const geo::Vector2& point, const Transform& t = Transform()) const noexcept override;
			Collider* Clone() const noexcept override;
			virtual geo::Vector2 GetCenter() const noexcept override;
			bool operator==(const Collider& c) const noexcept override;
			bool operator!=(const Collider& c) const noexcept override;
			geo::Vector2 Max() const noexcept override;
			geo::Vector2 Min() const noexcept override;
			geo::Vector2 GetPoint(size_t index) const;
			virtual std::vector<geo::Vector2> GetPoints(const Transform& t = Transform()) const noexcept override;
			geo::Vector2 GetNormal(size_t index) const;
			std::vector<geo::Vector2> GetNormals() const noexcept;
			geo::Vector2 SupportPoint(geo::Vector2 direction) const noexcept;
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
