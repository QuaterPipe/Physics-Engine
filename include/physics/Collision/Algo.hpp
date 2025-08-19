#pragma once
#include "Collision.hpp"

namespace physics::algo
{
	Manifold AABBCollision(
		const BoxCollider* a,
		const BoxCollider* b
	);

	Manifold PointMatrixCircleCollision(
		const PointMatrixCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb, bool flipped = false
	);

	Manifold PointMatrixBoxCollision(
		const PointMatrixCollider* a, const Transform& ta,
		const BoxCollider* b, const Transform& tb, bool flipped = false
	);

	Manifold PointMatrixPolygonCollision(
		const PointMatrixCollider* a, const Transform& ta,
		const PolygonCollider* b, const Transform& tb, bool flipped = false
	);

	Manifold PointMatrixMeshCollision(
		const PointMatrixCollider* a, const Transform& ta,
		const MeshCollider* b, const Transform& tb, bool flipped = false
	);

	Manifold PolygonCircleCollision(
		const PolygonCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb, bool flipped = false
	);

	Manifold PolygonBoxCollision(
		const PolygonCollider* a, const Transform& ta,
		const BoxCollider* b, const Transform& tb, bool flipped = false
	);

	Manifold PolygonMeshCollision(
		const PolygonCollider* a, const Transform& ta,
		const MeshCollider* b, const Transform& tb, bool flipped = false
	);

	Manifold CircleCircleCollision(
		const CircleCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb, bool flipped = false
	);

	Manifold CircleBoxCollision(
		const CircleCollider* a, const Transform& ta,
		const BoxCollider* b, const Transform& tb, bool flipped = false
	);

	Manifold CircleMeshCollision(
		const CircleCollider* a, const Transform& ta,
		const MeshCollider* b, const Transform& tb, bool flipped = false
	);

	Manifold BoxMeshCollision(
		const BoxCollider* a, const Transform& ta,
		const MeshCollider* b, const Transform& tb, bool flipped = false
	);

	Manifold PolygonPolygonCollision(
		const PolygonCollider* a, const Transform& ta,
		const PolygonCollider* b, const Transform& tb, bool flipped = false
	);

	Manifold BoxBoxCollision(
		const BoxCollider* a, const Transform& ta,
		const BoxCollider* b, const Transform& tb, bool flipped = false
	);

	Manifold MeshMeshCollision(
		const MeshCollider* a, const Transform& ta,
		const MeshCollider* b, const Transform& tb, bool flipped = false
	);

	Manifold PointMatrixPointMatrixCollision(
		const PointMatrixCollider* a, const Transform& ta,
		const PointMatrixCollider* b, const Transform& tb, bool flipped = false
	);

	bool VectorInPolygon(
		const Vector2* points,
		const Vector2& b, size_t pointsSize
	);

	Vector3 SAT(Vector2* aPoints, size_t aSize, Vector2* bPoints, size_t bSize) noexcept;
}