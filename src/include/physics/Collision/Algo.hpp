#pragma once
#include "Collision.hpp"
namespace physics::algo
{
	struct LineCollision
	{
		geo::Line a;
		geo::Line b;
		geo::Vector poi;
	};

	struct PolygonCollision
	{
		std::vector<LineCollision> collisions;
		size_t VertexesInACount = 0;
		size_t VertexesInBCount = 0;
	};

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

	static bool VectorInPolygon(
		const PolygonCollider* a, const Transform& ta,
		const geo::Vector& b
	);

	static bool LinePassesThroughCircle(
		const geo::Line& a, const CircleCollider* b,
		const Transform& tb
	);

	static CollisionPoints LineInCircle(
		const CircleCollider* a, const Transform& ta,
		const geo::Line& b
	);

	bool VectorInCircle(const geo::Vector& a, const CircleCollider* b,
		const Transform& tb
	);

	static bool CircleInsideCircle(
		const CircleCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	);

	static CollisionPoints CircleInsidePolygon(
		const PolygonCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	);

	static CollisionPoints PolygonInsideCircle(
		const PolygonCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	);

	static CollisionPoints CircleCenterInPolygon(
		const PolygonCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	);

	static CollisionPoints PolygonLineInCircle(
		const PolygonCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	);

	static CollisionPoints PolygonVertexInCircle(
		const PolygonCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	);

	static CollisionPoints PolygonVertexInPolygon(
		const PolygonCollider* a, const Transform& ta,
		const PolygonCollider* b, const Transform* tb,
		const PolygonCollision& collision
	);

	static CollisionPoints PolygonInsidePolygon(
		const PolygonCollider* a, const Transform& ta,
		const PolygonCollider* b, const Transform& tb
	);

	static std::vector<geo::Vector> GetIntersectionsBetweenTwoPolygons(
		const PolygonCollider* a, const Transform& ta,
		const PolygonCollider* b, const Transform& tb
	);

	static void GenerateLineCollision(
		const PolygonCollider* a, const Transform& ta,
		const PolygonCollider* b, const Transform& tb,
		PolygonCollision& collisions
	);
}