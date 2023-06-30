#pragma once
#include "geometry/main.hpp"
#include "SFML/Graphics.hpp"
#include "Transform.hpp"

namespace physics
{
	struct BoxCollider;
	struct Collider;
	struct CircleCollider;
	struct PointCollider;
	struct PolygonCollider;
	struct MeshCollider;
	struct CollisionPoints;

	struct CollisionPoints
	{
		//the points where the two objects touch
		std::vector<geo::Vector2> points;
		// the normal direction
		geo::Vector2 normal;
		// The distance between the two most shallow points.
		f64 depth = 0;
		bool hasCollision = false;
		inline bool operator==(const CollisionPoints& other) const noexcept
		{
			return points == other.points && normal == other.normal && hasCollision == other.hasCollision;
		}
		inline bool operator!=(const CollisionPoints& other) const noexcept
		{
			return !operator==(other);
		}
	};

	struct Collider
	{
		Collider() noexcept;
		virtual Collider* Clone() const noexcept;
		virtual ~Collider() noexcept;
		virtual BoxCollider BoundingBox(const Transform& t = Transform()) const noexcept = 0;
		virtual geo::Vector2 GetCenter() const noexcept = 0;
		virtual std::vector<geo::Vector2> GetPoints(const Transform& t = Transform()) const noexcept = 0;
		virtual geo::Vector2 Max() const noexcept = 0;
		virtual geo::Vector2 Min() const noexcept = 0;
		virtual bool operator==(const Collider& c) const noexcept = 0;
		virtual bool operator!=(const Collider& c) const noexcept = 0;
		virtual CollisionPoints TestCollision(
			const Transform& transform,
			const Collider* collider,
			const Transform& colliderTransform) const noexcept = 0;
		virtual CollisionPoints TestCollision(
			const Transform& transform,
			const CircleCollider* collider,
			const Transform& colliderTransform) const noexcept = 0;
		virtual CollisionPoints TestCollision(
			const Transform& transform,
			const PolygonCollider* collider,
			const Transform& colliderTransform) const noexcept = 0;
		virtual CollisionPoints TestCollision(
			const Transform& transform,
			const BoxCollider* collider,
			const Transform& colliderTransform) const noexcept = 0;
		virtual CollisionPoints TestCollision(
			const Transform& transform,
			const MeshCollider* collider,
			const Transform& colliderTransform) const noexcept = 0;
		virtual CollisionPoints TestCollision(
			const Transform& transform,
			const PointCollider* collider,
			const Transform& colliderTransform) const noexcept = 0;
	};
}