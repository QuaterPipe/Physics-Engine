#pragma once
#include "../geometry/main.hpp"
#include "Serializable.hpp"
#include "Transform.hpp"
namespace physics
{
	struct BoxCollider;
	struct Collider;
	struct CircleCollider;
	struct PolygonCollider;
	struct MeshCollider;
	struct CollisionPoints;

	struct CollisionPoints
	{
		geometry::Vector a;
		geometry::Vector b;
		geometry::Vector normal;
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
		virtual Collider* Clone() const = 0;
		virtual ~Collider() noexcept;
		virtual geometry::Vector GetCenter() const noexcept = 0;
		virtual geometry::Vector Max() const noexcept = 0;
		virtual geometry::Vector Min() const noexcept = 0;
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
	};
}