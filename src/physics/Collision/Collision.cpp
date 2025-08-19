#include "physics/Collision/Collision.hpp"
#include "physics/Collision/Algo.hpp"
#include <iostream>

namespace physics
{

	Manifold CircleCollider::TestCollision(const Transform& transform,
		const Collider* collider,
		const Transform& colliderTransform) const noexcept
	{
		Manifold c = collider->TestCollision(colliderTransform, this, transform);
		c.normal = -c.normal;
		return c;
	}

	Manifold CircleCollider::TestCollision(const Transform& transform,
		const CircleCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::CircleCircleCollision(this, transform, collider, colliderTransform);
	}

	Manifold CircleCollider::TestCollision(const Transform& transform,
		const PolygonCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::PolygonCircleCollision(collider, colliderTransform, this, transform, true);
	}

	Manifold CircleCollider::TestCollision(const Transform &transform,
		const BoxCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::CircleBoxCollision(this, transform, collider, colliderTransform);
	}

	Manifold CircleCollider::TestCollision(const Transform &transform,
		const MeshCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::CircleMeshCollision(this, transform, collider, colliderTransform);
	}

	Manifold CircleCollider::TestCollision(const Transform& transform,
		const PointMatrixCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::PointMatrixCircleCollision(collider, colliderTransform, this, transform, true);
	}

	Manifold PolygonCollider::TestCollision(const Transform& transform,
		const Collider* collider,
		const Transform& colliderTransform) const noexcept
	{
		Manifold c = collider->TestCollision(colliderTransform, this, transform);
		c.normal = -c.normal;
		return c;
	}

	Manifold PolygonCollider::TestCollision(const Transform& transform,
		const CircleCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::PolygonCircleCollision(this, transform, collider, colliderTransform);
	}

	Manifold PolygonCollider::TestCollision(const Transform& transform,
		const PolygonCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::PolygonPolygonCollision(this, transform, collider, colliderTransform);
	}

	Manifold PolygonCollider::TestCollision(const Transform& transform,
		const BoxCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::PolygonBoxCollision(this, transform, collider, colliderTransform);
	}

	Manifold PolygonCollider::TestCollision(const Transform& transform,
		const MeshCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::PolygonMeshCollision(this, transform, collider, colliderTransform);
	}

	Manifold PolygonCollider::TestCollision(const Transform& transform,
		const PointMatrixCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::PointMatrixPolygonCollision(collider, colliderTransform, this, transform, true);
	}

	Manifold BoxCollider::TestCollision(const Transform& transform,
		const Collider* collider,
		const Transform& colliderTransform) const noexcept
	{
		Manifold c = collider->TestCollision(colliderTransform, this, transform);
		c.normal = -c.normal;
		return c;
	}

	Manifold BoxCollider::TestCollision(const Transform& transform,
		const CircleCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::CircleBoxCollision(collider, colliderTransform, this, transform, true);
	}

	Manifold BoxCollider::TestCollision(const Transform& transform,
		const PolygonCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::PolygonBoxCollision(collider, colliderTransform, this, transform, true);
	}

	Manifold BoxCollider::TestCollision(const Transform& transform,
		const BoxCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::BoxBoxCollision(this, transform, collider, colliderTransform);
	}

	Manifold BoxCollider::TestCollision(const Transform& transform,
		const MeshCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::BoxMeshCollision(this, transform, collider, colliderTransform);
	}

	Manifold BoxCollider::TestCollision(const Transform& transform,
		const PointMatrixCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::PointMatrixBoxCollision(collider, colliderTransform, this, transform, true);
	}

	Manifold MeshCollider::TestCollision(const Transform& transform,
		const Collider* collider,
		const Transform& colliderTransform
	) const noexcept
	{
		Manifold c = collider->TestCollision(colliderTransform, this, transform);
		c.normal = -c.normal;
		return c;
	}

	Manifold MeshCollider::TestCollision(const Transform& transform,
		const CircleCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::CircleMeshCollision(collider, colliderTransform, this, transform, true);
	}

	Manifold MeshCollider::TestCollision(const Transform& transform,
		const PolygonCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::PolygonMeshCollision(collider, colliderTransform, this, transform, true);
	}

	Manifold MeshCollider::TestCollision(const Transform& transform,
		const BoxCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::BoxMeshCollision(collider, colliderTransform, this, transform, true);
	}

	Manifold MeshCollider::TestCollision(const Transform& transform,
		const MeshCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::MeshMeshCollision(this, transform, collider, colliderTransform);
	}

	Manifold MeshCollider::TestCollision(const Transform& transform,
		const PointMatrixCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::PointMatrixMeshCollision(collider, colliderTransform, this, transform, true);
	}

	Manifold PointMatrixCollider::TestCollision(const Transform& transform,
		const CircleCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::PointMatrixCircleCollision(this, transform, collider, colliderTransform);
	}

	Manifold PointMatrixCollider::TestCollision(const Transform& transform,
		const BoxCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::PointMatrixBoxCollision(this, transform, collider, colliderTransform);
	}

	Manifold PointMatrixCollider::TestCollision(const Transform& transform,
		const PolygonCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::PointMatrixPolygonCollision(this, transform, collider, colliderTransform);
	}

	Manifold PointMatrixCollider::TestCollision(const Transform& transform,
		const MeshCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::PointMatrixMeshCollision(this, transform, collider, colliderTransform);
	}

	Manifold PointMatrixCollider::TestCollision(const Transform& transform,
		const PointMatrixCollider* collider,
		const Transform& colliderTransform) const noexcept
	{
		return algo::PointMatrixPointMatrixCollision(this, transform, collider, colliderTransform);
	}
}