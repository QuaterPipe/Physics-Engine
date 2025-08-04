#include "physics/Collision/Algo.hpp"
#include <iostream>
#define MAX std::numeric_limits<f64>::max()
#define MIN std::numeric_limits<f64>::min()

namespace physics::algo
{
    Manifold CircleCircleCollision(
		const CircleCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb, bool flipped
	)
	{
		Manifold c;
		c.hasCollision = false;
		if (!a || !b) {return c;}
		const Vector2 ACenter = ta.TransformVector(a->center);
		const Vector2 BCenter = tb.TransformVector(b->center);
		const f64 r = DistanceSquared(ACenter, BCenter);
		// If the sum of their radii is greater than or equal to the distance between their centers
		const f64 aRadius = a->radius * Max(ta.GetScale().x, ta.GetScale().y);
		const f64 bRadius = b->radius * Max(tb.GetScale().x, tb.GetScale().y);
		if (SQRD(aRadius + bRadius) >= r)
		{
			f64 d = FastSqrt(r);
			Vector2 ca = ACenter.Lerp(BCenter, aRadius / d);
			Vector2 cb = BCenter.Lerp(ACenter, bRadius / d);
			c.depth = ca != cb ? Distance(ca, cb) : std::max(aRadius, bRadius);
			c.normal = ca != cb ? ca - cb : Vector2(0, 1);
			c.normal.Normalize();
			c.points[0] = ca;
			c.points[1] = cb;
			c.hasCollision = true;
			c.pointCount = 2;
			if (flipped)
				c.normal = -c.normal;
		}
		return c;
	}

	Manifold CircleBoxCollision(
		const CircleCollider* a, const Transform& ta,
		const BoxCollider* b, const Transform& tb, bool flipped
	)
	{
		Manifold c;
		if (!a || !b ) {return c;}
		//easier to  collision points as a PolygonCollider
		PolygonCollider bb = PolygonCollider(*b);
		return PolygonCircleCollision(&bb, tb, a, ta, !flipped);
	}

	Manifold CircleMeshCollision(
		const CircleCollider* a, const Transform& ta,
		const MeshCollider* b, const Transform& tb, bool flipped
	)
	{
		Manifold c;
		if (!a || !b)
			return c;
		f64 avg = 0;
		for (const Collider* ptr : b->colliders)
		{
			Manifold tmp = ptr->TestCollision(tb, a, ta);
			if (tmp.hasCollision)
			{
				avg += tmp.depth;
				c.hasCollision = true;
				if (c.depth < tmp.depth)
				{
					c.depth = tmp.depth;
					c.normal = -tmp.normal;
				}
				for (size_t i = 0 ; i < tmp.pointCount; i++)
				{
					if (c.pointCount < MAX_MANIFOLD_POINT_COUNT)
						c.points[c.pointCount++] = tmp.points[i];
				}
			}
		}
		if (avg)
			c.depth = avg / (f64)c.pointCount;
		if (flipped)
			c.normal = -c.normal;
		return c;
	}
}