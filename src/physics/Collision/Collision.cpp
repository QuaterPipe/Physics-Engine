#include "physics/Collision/Collision.hpp"
#include "physics/Collision/Algo.hpp"
#include <iostream>

namespace physics
{

	CollisionPoints CircleCollider::TestCollision(const Transform& transform,
		const Collider* collider,
		const Transform& colliderTransform) const noexcept
	{
		CollisionPoints c = collider->TestCollision(colliderTransform, this, transform);
		c.normal = -c.normal;
		return c;
	}

	CollisionPoints CircleCollider::TestCollision(const Transform& transform,
		const CircleCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::CircleCircleCollision(this, transform, collider, colliderTransform);
	}

	CollisionPoints CircleCollider::TestCollision(const Transform& transform,
		const PolygonCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::PolygonCircleCollision(collider, colliderTransform, this, transform, true);
	}

	CollisionPoints CircleCollider::TestCollision(const Transform &transform,
		const BoxCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::CircleBoxCollision(this, transform, collider, colliderTransform);
	}

	CollisionPoints CircleCollider::TestCollision(const Transform &transform,
		const MeshCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::CircleMeshCollision(this, transform, collider, colliderTransform);
	}

	CollisionPoints PolygonCollider::TestCollision(const Transform& transform,
		const Collider* collider,
		const Transform& colliderTransform) const noexcept
	{
		CollisionPoints c = collider->TestCollision(colliderTransform, this, transform);
		c.normal = -c.normal;
		return c;
	}

	CollisionPoints PolygonCollider::TestCollision(const Transform& transform,
		const CircleCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::PolygonCircleCollision(this, transform, collider, colliderTransform);
	}

	CollisionPoints PolygonCollider::TestCollision(const Transform& transform,
		const PolygonCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::PolygonPolygonCollision(this, transform, collider, colliderTransform);
	}

	CollisionPoints PolygonCollider::TestCollision(const Transform& transform,
		const BoxCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::PolygonBoxCollision(this, transform, collider, colliderTransform);
	}

	CollisionPoints PolygonCollider::TestCollision(const Transform& transform,
		const MeshCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::PolygonMeshCollision(this, transform, collider, colliderTransform);
	}

	CollisionPoints BoxCollider::TestCollision(const Transform& transform,
		const Collider* collider,
		const Transform& colliderTransform) const noexcept
	{
		CollisionPoints c = collider->TestCollision(colliderTransform, this, transform);
		c.normal = -c.normal;
		return c;
	}

	CollisionPoints BoxCollider::TestCollision(const Transform& transform,
		const CircleCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::CircleBoxCollision(collider, colliderTransform, this, transform, true);
	}

	CollisionPoints BoxCollider::TestCollision(const Transform& transform,
		const PolygonCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::PolygonBoxCollision(collider, colliderTransform, this, transform, true);
	}

	CollisionPoints BoxCollider::TestCollision(const Transform& transform,
		const BoxCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::BoxBoxCollision(this, transform, collider, colliderTransform);
	}

	CollisionPoints BoxCollider::TestCollision(const Transform& transform,
		const MeshCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::BoxMeshCollision(this, transform, collider, colliderTransform);
	}

	CollisionPoints MeshCollider::TestCollision(const Transform& transform,
		const Collider* collider,
		const Transform& colliderTransform
	) const noexcept
	{
		CollisionPoints c = collider->TestCollision(colliderTransform, this, transform);
		c.normal = -c.normal;
		return c;
	}

	CollisionPoints MeshCollider::TestCollision(const Transform& transform,
		const CircleCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::CircleMeshCollision(collider, colliderTransform, this, transform, true);
	}

	CollisionPoints MeshCollider::TestCollision(const Transform& transform,
		const PolygonCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::PolygonMeshCollision(collider, colliderTransform, this, transform, true);
	}

	CollisionPoints MeshCollider::TestCollision(const Transform& transform,
		const BoxCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::BoxMeshCollision(collider, colliderTransform, this, transform, true);
	}

	CollisionPoints MeshCollider::TestCollision(const Transform& transform,
		const MeshCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::MeshMeshCollision(this, transform, collider, colliderTransform);
	}
}