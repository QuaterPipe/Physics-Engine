#include "physics/Collision/Algo.hpp"
#include <iostream>
#define MAX std::numeric_limits<f64>::max()
#define MIN std::numeric_limits<f64>::min()

namespace physics::algo
{
    CollisionPoints CircleCircleCollision(
		const CircleCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		c.hasCollision = false;
		if (!a || !b) {return c;}
		const geo::Vector2 ACenter = ta.TransformVector(a->center);
		const geo::Vector2 BCenter = tb.TransformVector(b->center);
		const f64 r = geo::DistanceSquared(ACenter, BCenter);
		// If the sum of their radii is greater than or equal to the distance between their centers
		if (SQRD(a->radius + b->radius) >= r)
		{
			geo::Line l(ACenter, BCenter);
			geo::Vector2 ca = l.GetVectorAlongLine(a->radius);
			geo::Vector2 cb = l.GetVectorAlongLine(b->radius, false);
			c.depth = geo::Distance(ca, cb);
			c.normal = ca - cb;
			c.normal.Normalize();
			c.points.push_back(ca);
			c.points.push_back(cb);
			c.hasCollision = true;
		}
		return c;
	}

	CollisionPoints CircleBoxCollision(
		const CircleCollider* a, const Transform& ta,
		const BoxCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		if (!a || !b ) {return c;}
		//easier to  collision points as a PolygonCollider
		PolygonCollider bb = PolygonCollider(*b);
		return PolygonCircleCollision(&bb, tb, a, ta);
	}

	CollisionPoints CircleMeshCollision(
		const CircleCollider* a, const Transform& ta,
		const MeshCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		if (!a || !b)
			return c;
		f64 avg = 0;
		for (const Collider* ptr : b->colliders)
		{
			CollisionPoints tmp = ptr->TestCollision(tb, a, ta);
			if (tmp.hasCollision)
			{
				avg += tmp.depth;
				c.hasCollision = true;
				if (c.depth < tmp.depth)
				{
					c.depth = tmp.depth;
					c.normal = tmp.normal;
				}
				for (auto p : tmp.points)
					c.points.push_back(p);
			}
		}
		if (avg)
			c.depth = avg / (f64)c.points.size();
		return c;
	}
}