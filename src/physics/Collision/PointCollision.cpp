#include "../../include/physics/Collision/Algo.hpp"
#include <iostream>
#define MAX std::numeric_limits<f64>::max()
#define MIN std::numeric_limits<f64>::min()

namespace physics::algo
{
    CollisionPoints PointCircleCollision(
		const PointCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		c.hasCollision = false;
		if (!a || !b) {return c;}
		const geo::Vector2 BCenter = tb.TransformVector(b->center);
		const geo::Vector2 APos = ta.TransformVector(a->position);
		if (geo::DistanceSquared(BCenter, APos) <= SQRD(b->radius))
		{
			c.a = APos;
			geo::Vector2 tmp = (APos - BCenter).Normalized();
			c.b = tmp * b->radius + BCenter;
			c.depth = geo::Distance(c.a, c.b);
			c.normal = (c.a - c.b).Normalized();
			c.hasCollision = true;
		}
		return c;
	}

	CollisionPoints PointPolygonCollision(
		const PointCollider* a, const Transform& ta,
		const PolygonCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		c.hasCollision = false;
		if (!a || !b) return c;
		geo::Vector2 APos = ta.TransformVector(a->position);
		if (!VectorInPolygon(b, tb, APos))
			return c;
		std::vector<geo::Vector2> BPoints;
		for (const geo::Vector2& v: b->points)
			BPoints.push_back(tb.TransformVector(v + b->pos));
		geo::Vector2 closest = geo::Vector2::Infinity;
		for (size_t i = 0; i < BPoints.size(); i++)
		{
			geo::Line l(BPoints[i], BPoints[(i + 1) % BPoints.size()]);
			geo::Vector2 p = geo::Vector2::Projection(APos, l);
			if (geo::DistanceSquared(closest, APos) > geo::DistanceSquared(p, APos))
			{
				if (l.VectorIsOnLine(p))
					closest = p;
			}
		}
		c.a = APos;
		c.b = closest;
		c.depth = geo::Distance(c.a, c.b);
		c.normal = (c.a - c.b).Normalized();
		c.hasCollision = true;
		return c;
	}

	CollisionPoints PointBoxCollision(
		const PointCollider* a, const Transform& ta,
		const BoxCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		c.hasCollision = false;
		if (!a || !b) return c;
		PolygonCollider bb = PolygonCollider(*b);
		return PointPolygonCollision(a, ta, &bb, tb);
	}

	CollisionPoints PointMeshCollision(
		const PointCollider* a, const Transform& ta,
		const MeshCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		c.hasCollision = false;
		if (!a || !b) {return c;}
		for (const Collider* ptr: b->colliders)
		{
			c = ptr->TestCollision(tb, a, ta);
			if (c.hasCollision) {return c;}
		}
		return c;
	}

	CollisionPoints PointPointCollision(
		const PointCollider* a, const Transform& ta,
		const PointCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		c.hasCollision = false;
		if (!a || !b) {return c;}
		if (geo::DistanceSquared(ta.TransformVector(a->position), tb.TransformVector(b->position)) < SQRD(EPSILON))
		{
			c.a = ta.TransformVector(a->position);
			c.b = tb.TransformVector(b->position);
			c.depth = geo::Distance(c.a, c.b);
			c.normal = (c.a - c.b).Normalized();
			c.hasCollision = true;
		}
		return c;
	}
}