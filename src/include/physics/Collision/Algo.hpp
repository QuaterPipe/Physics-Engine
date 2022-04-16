	#include "Collision.hpp"
namespace physics::algo
{
	CollisionPoints PointPointCollision(
		const PointCollider* a, const Transform& ta,
		const PointCollider* b, const Transform& tb
	);

	CollisionPoints PointPolygonCollision(
		const PointCollider* a, const Transform& ta,
		const PolygonCollider* b, const Transform& tb
	);

	CollisionPoints PointCircleCollision(
		const PointCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	);

	CollisionPoints PointBoxCollision(
		const PointCollider* a, const Transform& ta,
		const BoxCollider* b, const Transform& tb
	);

	CollisionPoints PointMeshCollision(
		const PointCollider* a, const Transform& ta,
		const MeshCollider* b, const Transform& tb
	);

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

	std::vector<geometry::Vector> GetIntersectionsBetweenTwoPolygons(
		const PolygonCollider* a, const Transform& ta,
		const PolygonCollider* b, const Transform& tb
	);
}