#pragma once
#include "physics/Geometry/main.hpp"
#include "SFML/Graphics.hpp"
#include "Transform.hpp"
#define MAX_MANIFOLD_POINT_COUNT 5

namespace physics
{
	struct BoxCollider;
	struct Collider;
	struct CircleCollider;
	struct PolygonCollider;
	struct MeshCollider;
	struct Manifold;

	struct Manifold
	{
		//the points where the two objects touch
		Vector2 points[MAX_MANIFOLD_POINT_COUNT];
		// the amount of points in the manifold
		size_t pointCount = 0;
		// the normal direction
		Vector2 normal;
		// The distance between the two most shallow points.
		f64 depth = 0;
		bool hasCollision = false;
		inline bool operator==(const Manifold& other) const noexcept
		{
			return points == other.points && normal == other.normal && hasCollision == other.hasCollision;
		}
		inline bool operator!=(const Manifold& other) const noexcept
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
		virtual bool Contains(const Vector2& point, const Transform& t = Transform()) const noexcept = 0;
		virtual f64 CrossSectionalArea(const Vector2& direction) const noexcept = 0;
		virtual Vector2 GetCenter() const noexcept = 0;
		virtual std::vector<Vector2> GetPoints(const Transform& t = Transform()) const noexcept = 0;
		virtual Vector2 Max() const noexcept = 0;
		virtual Vector2 Min() const noexcept = 0;
		virtual bool operator==(const Collider& c) const noexcept = 0;
		virtual bool operator!=(const Collider& c) const noexcept = 0;
		virtual Manifold TestCollision(
			const Transform& transform,
			const Collider* collider,
			const Transform& colliderTransform) const noexcept = 0;
		virtual Manifold TestCollision(
			const Transform& transform,
			const CircleCollider* collider,
			const Transform& colliderTransform) const noexcept = 0;
		virtual Manifold TestCollision(
			const Transform& transform,
			const PolygonCollider* collider,
			const Transform& colliderTransform) const noexcept = 0;
		virtual Manifold TestCollision(
			const Transform& transform,
			const BoxCollider* collider,
			const Transform& colliderTransform) const noexcept = 0;
		virtual Manifold TestCollision(
			const Transform& transform,
			const MeshCollider* collider,
			const Transform& colliderTransform) const noexcept = 0;
	};
}