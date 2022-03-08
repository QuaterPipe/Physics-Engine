#include "../include/physics/Collision.hpp"
#include "../include/physics/Algo.hpp"
#include "../include/physics/OstreamOverloads.hpp"
#include <iostream>

namespace physics
{

	CollisionPoints CircleCollider::TestCollision(const Transform& transform,
		const Collider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return collider->TestCollision(colliderTransform, this, transform);
	}

	CollisionPoints CircleCollider::TestCollision(const Transform& transform,
		const CircleCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::FindCircleCircleCollisionPoints(this, transform, collider, colliderTransform);
	}

	CollisionPoints CircleCollider::TestCollision(const Transform& transform,
		const PolygonCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::FindPolygonCircleCollisionPoints(collider, colliderTransform, this, transform);
	}

	CollisionPoints CircleCollider::TestCollision(const Transform &transform,
		const BoxCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::FindCircleBoxCollisionPoints(this, transform, collider, colliderTransform);
	}

	CollisionPoints CircleCollider::TestCollision(const Transform &transform,
		const MeshCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::FindCircleMeshCollisionPoints(this, transform, collider, colliderTransform);
	}

	CollisionPoints PolygonCollider::TestCollision(const Transform& transform,
		const Collider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return collider->TestCollision(colliderTransform, this, transform);
	}

	CollisionPoints PolygonCollider::TestCollision(const Transform& transform,
		const CircleCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::FindPolygonCircleCollisionPoints(this, transform, collider, colliderTransform);
	}

	CollisionPoints PolygonCollider::TestCollision(const Transform& transform,
		const PolygonCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::FindPolygonPolygonCollisionPoints(this, transform, collider, colliderTransform);
	}

	CollisionPoints PolygonCollider::TestCollision(const Transform& transform,
		const BoxCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::FindPolygonBoxCollisionPoints(this, transform, collider, colliderTransform);
	}

	CollisionPoints PolygonCollider::TestCollision(const Transform& transform,
		const MeshCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::FindPolygonMeshCollisionPoints(this, transform, collider, colliderTransform);
	}

	CollisionPoints BoxCollider::TestCollision(const Transform& transform,
		const Collider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return collider->TestCollision(colliderTransform, this, transform);
	}

	CollisionPoints BoxCollider::TestCollision(const Transform& transform,
		const CircleCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::FindCircleBoxCollisionPoints(collider, colliderTransform, this, transform);
	}

	CollisionPoints BoxCollider::TestCollision(const Transform& transform,
		const PolygonCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::FindPolygonBoxCollisionPoints(collider, colliderTransform, this, transform);
	}

	CollisionPoints BoxCollider::TestCollision(const Transform& transform,
		const BoxCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::FindBoxBoxCollisionPoints(this, transform, collider, colliderTransform);
	}

	CollisionPoints BoxCollider::TestCollision(const Transform& transform,
		const MeshCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::FindBoxMeshCollisionPoints(this, transform, collider, colliderTransform);
	}

	CollisionPoints MeshCollider::TestCollision(const Transform& transform,
		const Collider* collider,
		const Transform& colliderTransform
	) const noexcept
	{
		return collider->TestCollision(colliderTransform, this, transform);
	}

	CollisionPoints MeshCollider::TestCollision(const Transform& transform,
		const CircleCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::FindCircleMeshCollisionPoints(collider, colliderTransform, this, transform);
	}

	CollisionPoints MeshCollider::TestCollision(const Transform& transform,
		const PolygonCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::FindPolygonMeshCollisionPoints(collider, colliderTransform, this, transform);
	}

	CollisionPoints MeshCollider::TestCollision(const Transform& transform,
		const BoxCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::FindBoxMeshCollisionPoints(collider, colliderTransform, this, transform);
	}

	CollisionPoints MeshCollider::TestCollision(const Transform& transform,
		const MeshCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::FindMeshMeshCollisionPoints(this, transform, collider, colliderTransform);
	}
}