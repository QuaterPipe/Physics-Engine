#include "physics/Collision/Algo.hpp"
#include <iostream>
#define MAX std::numeric_limits<f64>::max()
#define MIN std::numeric_limits<f64>::min()

namespace physics::algo
{

    CollisionPoints AABBCollision(
        const BoxCollider* a,
        const BoxCollider* b
    )
    {
        CollisionPoints c;
        if (!a || !b) { return c; }
        if (a->x < b->x + b->width && b->width < a->x + a->width &&
            a->y < b->y + b->height && b->y < a->y + a->height)
        {
            c.hasCollision = true;
            if (a->x <= b->x)
            {
                c.depth = b->x - a->x;
                c.normal.Set(-1, 0);
                if (a->y <= b->y)
                {
                    c.points.push_back(geo::Vector2(a->x + a->width, a->y + a->height));
                    c.points.push_back(geo::Vector2(b->x, b->y));
                    if (c.depth < b->y - a->y)
                    {
                        c.depth = b->y - a->y;
                        c.normal.Set(0, -1);
                    }
                }
                if (a->y >= b->y)
                {
                    c.points.push_back(geo::Vector2(a->x + a->width, a->y));
                    c.points.push_back(geo::Vector2(b->x, b->y + b->height));
                    if (c.depth < a->y - b->y)
                    {
                        c.depth = a->y - b->y;
                        c.normal.Set(0, 1);
                    }
                }
            }
            if (a->x >= b->x)
            {
                c.depth = a->x - b->x;
                c.normal.Set(1, 0);
                if (a->y <= b->y)
                {
                    c.points.push_back(geo::Vector2(a->x, a->y + a->height));
                    c.points.push_back(geo::Vector2(b->x + b->width, b->y));
                    if (c.depth < b->y - a->y)
                    {
                        c.depth = b->y - a->y;
                        c.normal.Set(0, -1);
                    }
                }
                if (a->y >= b->y)
                {
                    c.points.push_back(geo::Vector2(a->x, a->y));
                    c.points.push_back(geo::Vector2(b->x + b->width, b->y + b->height));
                    if (c.depth < a->y - b->y)
                    {
                        c.depth = a->y - b->y;
                        c.normal.Set(0, 1);
                    }
                }
            }
        }
        return c;
    }
    CollisionPoints BoxBoxCollision(
		const BoxCollider* a, const Transform& ta,
		const BoxCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		if (!a || !b) {return c;}
        // is AABB
        if (!ta.rotation.Angle() && !tb.rotation.Angle())
        {
            BoxCollider aCpy(*a);
            BoxCollider bCpy(*b);
            aCpy.pos += ta.position;
            bCpy.pos += tb.position;
            aCpy.pos -= ta.centerOfRotation;
            bCpy.pos -= tb.centerOfRotation;
            aCpy.width *= ta.scale[0][0];
            aCpy.height *= ta.scale[1][1];
            bCpy.width *= tb.scale[0][0];
            bCpy.height *= tb.scale[1][1];
            return AABBCollision(&aCpy, &bCpy);
        }
		PolygonCollider bb = PolygonCollider(*b);
		PolygonCollider aa = PolygonCollider(*a);
		return PolygonPolygonCollision(&aa, ta, &bb, tb);
	}

	CollisionPoints BoxMeshCollision(
		const BoxCollider* a, const Transform& ta,
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