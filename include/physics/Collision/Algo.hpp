#pragma once
#include "Collision.hpp"

namespace physics::algo
{
	CollisionPoints PolygonCircleCollision(
		const PolygonCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	);

	CollisionPoints PolygonBoxCollision(
		const PolygonCollider* a, const Transform& ta,
		const BoxCollider* b, const Transform& tb
	);

	CollisionPoints PolygonMeshCollision(
		const PolygonCollider* a, const Transform& ta,
		const MeshCollider* b, const Transform& tb
	);


	CollisionPoints CircleCircleCollision(
		const CircleCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	);

	CollisionPoints CircleBoxCollision(
		const CircleCollider* a, const Transform& ta,
		const BoxCollider* b, const Transform& tb
	);

	CollisionPoints CircleMeshCollision(
		const CircleCollider* a, const Transform& ta,
		const MeshCollider* b, const Transform& tb
	);

	CollisionPoints BoxMeshCollision(
		const BoxCollider* a, const Transform& ta,
		const MeshCollider* b, const Transform& tb
	);

	CollisionPoints PolygonPolygonCollision(
		const PolygonCollider* a, const Transform& ta,
		const PolygonCollider* b, const Transform& tb
	);

	CollisionPoints BoxBoxCollision(
		const BoxCollider* a, const Transform& ta,
		const BoxCollider* b, const Transform& tb
	);

	CollisionPoints MeshMeshCollision(
		const MeshCollider* a, const Transform& ta,
		const MeshCollider* b, const Transform& tb
	);

	bool VectorInPolygon(
		const PolygonCollider* a, const Transform& ta,
		const geo::Vector2& b
	);

	bool VectorInCircle(const geo::Vector2& a, const CircleCollider* b,
		const Transform& tb
	);

	i32 Clip(geo::Vector2 n, f64 c, geo::Vector2* face);

	void FindIncidentFace(geo::Vector2* v, const PolygonCollider* refPoly,
		const Transform& refTransform, const PolygonCollider* incPoly,
		const Transform& incTransform, size_t refIndex
	);

	f64 FindAxisLeastPenetration(size_t* faceIndex, const PolygonCollider* a,
		const Transform& ta, const PolygonCollider* b, const Transform& tb
	);
}