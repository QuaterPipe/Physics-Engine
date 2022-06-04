#pragma once
#include "../../geometry/main.hpp"
#include "../../SFML/Graphics.hpp"
#include "../Base/Serializable.hpp"
#include "../Base/Component.hpp"
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
		// the deepest point into collider b that is a part of a.
		geo::Vector a;
		// the deepest poin into collider a that is a part of b.
		geo::Vector b;
		// it i sb - a(usually) is always pointint in the direction
		// to push a out of b in the shortest distance.
		geo::Vector normal;
		// The distance between the two deepest points.
		f64 depth = 0;
		bool hasCollision = false;
		inline bool operator==(const CollisionPoints& other) const noexcept
		{
			return a == other.a && b == other.b && normal == other.normal && hasCollision == other.hasCollision;
		}

		inline bool operator!=(const CollisionPoints& other) const noexcept
		{
			return !(a == other.a && b == other.b && normal == other.normal && hasCollision == other.hasCollision);
		}
	};

	struct Collider : public serialization::Serializable, public Hashable
	{
		virtual Collider* Clone() const noexcept;
		virtual ~Collider() noexcept;
		virtual BoxCollider BoundingBox(const Transform& t = Transform()) const noexcept = 0;
		virtual geo::Vector GetCenter() const noexcept = 0;
		virtual std::vector<geo::Vector> GetPoints(const Transform& t = Transform()) const noexcept = 0;
		virtual geo::Vector Max() const noexcept = 0;
		virtual geo::Vector Min() const noexcept = 0;
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