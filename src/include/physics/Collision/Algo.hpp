#pragma once
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

	bool VectorInPolygon(
		const PolygonCollider* a, const Transform& ta,
		const geo::Vector& b
	);

	geo::Vector PointOfIntersect(
		const PolygonCollider* a, const Transform &ta,
		const PolygonCollider* b, const Transform& tb
	);

	bool LinePassesThroughCircle(
		const geo::Line& a, const CircleCollider* b,
		const Transform& tb
	);

	CollisionPoints LineInCircle(
		const CircleCollider* a, const Transform& ta,
		const geo::Line& b
	);

	bool VectorInCircle(const geo::Vector& a, const CircleCollider* b,
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

	CollisionPoints PolygonInsideCircle(
		const PolygonCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	);

	CollisionPoints CircleCenterInPolygon(
		const PolygonCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	);

	CollisionPoints PolygonLineInCircle(
		const PolygonCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	);

	CollisionPoints PolygonVertexInCircle(
		const PolygonCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	);

	CollisionPoints SeparatingAxisCheck(
		const PolygonCollider* a, const Transform& ta,
		const PolygonCollider* b, const Transform& tb,
		const geo::Vector& orthogonal
	);
}