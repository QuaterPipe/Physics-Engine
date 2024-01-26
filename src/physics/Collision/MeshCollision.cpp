#include "physics/Collision/Algo.hpp"
#include <iostream>
#define MAX std::numeric_limits<f64>::max()
#define MIN std::numeric_limits<f64>::min()

namespace physics::algo
{
    Manifold MeshMeshCollision(
		const MeshCollider* a, const Transform& ta,
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