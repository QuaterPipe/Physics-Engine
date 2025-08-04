#include "physics/Collision/Algo.hpp"
#include <iostream>
#define MAX std::numeric_limits<f64>::max()
#define MIN std::numeric_limits<f64>::min()

namespace physics::algo
{

    Manifold AABBCollision(
        const BoxCollider* a,
        const BoxCollider* b
    )
    {
        Manifold c;
        c.hasCollision = false;
        if (!a || !b) { return c; }
        if (a->Overlaps(*b))
        {
            c.hasCollision = true;
            if (a->x <= b->x)
            {
                c.depth = (a->x + a->width) - b->x;
                c.normal.Set(-1, 0);
                if (a->y <= b->y)
                {
                    c.points[0] = Vector2(a->x + (a->width / 2), a->y + (a->height / 2));
                    c.points[1] = Vector2(b->x - (b->width / 2), b->y - (b->height / 2));
                    if (c.depth > (a->y + a->height) - b->y)
                    {
                        c.depth = (a->y + a->height) - b->y;
                        c.normal.Set(0, -1);
                    }
                    c.pointCount = 2;
                }
                else if (a->y > b->y)
                {
                    c.points[0] = Vector2(a->x + (a->width / 2), a->y - (a->height / 2));
                    c.points[1] = Vector2(b->x - (b->width / 2), b->y + (b->height / 2));
                    if (c.depth > ((b->y + b->height) - a->y))
                    {
                        c.depth = (b->y + b->height) - a->y;
                        c.normal.Set(0, 1);
                    }
                    c.pointCount = 2;
                }

            }
            else if (a->x >= b->x)
            {   
                c.depth = (b->x + b->width) - a->x;
                c.normal.Set(1, 0);
                if (a->y <= b->y)
                {
                    c.points[0] = Vector2(a->x - (a->width / 2), a->y + (a->height / 2));
                    c.points[1] = Vector2(b->x + (b->width / 2), b->y - (b->height / 2));
                    if (c.depth > ((a->y + a->height) - b->y))
                    {
                        c.depth = ((a->y + a->height) - b->y);
                        c.normal.Set(0, -1);
                    }
                    c.pointCount = 2;
                }
                else if (a->y > b->y)
                {
                    c.points[0] = Vector2(a->x - (a->width / 2), a->y - (a->height / 2));
                    c.points[1] = Vector2(b->x + (b->width / 2), b->y + (b->height / 2));
                    if (c.depth > ((b->y + b->height) - a->y))
                    {
                        c.depth = ((b->y + b->height) - a->y);
                        c.normal.Set(0, 1);
                    }
                    c.pointCount = 2;
                }
            }
        }
        return c;
    }
    Manifold BoxBoxCollision(
		const BoxCollider* a, const Transform& ta,
		const BoxCollider* b, const Transform& tb, bool flipped
	)
	{
		Manifold c;
		if (!a || !b) {return c;}
        // is AABB
        if (!ta.GetAngle() && !tb.GetAngle())
        {
            BoxCollider aCpy(*a);
            BoxCollider bCpy(*b);
            aCpy.pos += ta.GetPosition();
            bCpy.pos += tb.GetPosition();
            aCpy.pos += ta.GetCOM();
            bCpy.pos += tb.GetCOM();
            aCpy.dimensions *= ta.GetScale();
            bCpy.dimensions *= tb.GetScale();
            if (flipped)
            {
                BoxCollider tmp = aCpy;
                aCpy = bCpy;
                bCpy = tmp;
            }
            return AABBCollision(&aCpy, &bCpy);
        }
		PolygonCollider aa = PolygonCollider(*a);
		PolygonCollider bb = PolygonCollider(*b);
		return PolygonPolygonCollision(&aa, ta, &bb, tb, flipped);
	}

	Manifold BoxMeshCollision(
		const BoxCollider* a, const Transform& ta,
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
                for (size_t i = 0; i < tmp.pointCount; i++)
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