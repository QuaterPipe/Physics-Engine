#include "Collision.hpp"
using namespace physics;
namespace algo
{
	CollisionPoints FindPolygonCircleCollisionPoints(
		const PolygonCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	);

	CollisionPoints FindPolygonBoxCollisionPoints(
		const PolygonCollider* a, const Transform& ta,
		const BoxCollider* b, const Transform& tb
	);

	CollisionPoints FindPolygonMeshCollisionPoints(
		const PolygonCollider* a, const Transform& ta,
		const MeshCollider* b, const Transform& tb
	);

	CollisionPoints FindCircleCircleCollisionPoints(
		const CircleCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	);

	CollisionPoints FindCircleBoxCollisionPoints(
		const CircleCollider* a, const Transform& ta,
		const BoxCollider* b, const Transform& tb
	);

	CollisionPoints FindCircleMeshCollisionPoints(
		const CircleCollider* a, const Transform& ta,
		const MeshCollider* b, const Transform& tb
	);

	CollisionPoints FindBoxMeshCollisionPoints(
		const BoxCollider* a, const Transform& ta,
		const MeshCollider* b, const Transform& tb
	);

	CollisionPoints FindPolygonPolygonCollisionPoints(
		const PolygonCollider* a, const Transform& ta,
		const PolygonCollider* b, const Transform& tb
	);

	CollisionPoints FindBoxBoxCollisionPoints(
		const BoxCollider* a, const Transform& ta,
		const BoxCollider* b, const Transform& tb
	);

	CollisionPoints FindMeshMeshCollisionPoints(
		const MeshCollider* a, const Transform& ta,
		const MeshCollider* b, const Transform& tb
	);

	bool PolygonColliderVectorIsColliding(
		const PolygonCollider* a, const Transform& ta,
		const geometry::Vector& b
	);

	bool LinePassesThroughCircle(
		const geometry::Line& a, const CircleCollider* b,
		const Transform& tb
	);

	bool CircleInsideCircle(
		const CircleCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	);

	CollisionPoints CircleInsidePolygon(
		const PolygonCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	);

	CollisionPoints PolygonInsidePolygon(
		const PolygonCollider* a, const Transform& ta,
		const PolygonCollider* b, const Transform& tb
	);
}