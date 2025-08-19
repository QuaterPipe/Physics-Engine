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
        c.depth = -std::numeric_limits<f64>::infinity();
        for (const Collider* ptr : b->colliders)
        {
            Manifold tmp = ptr->TestCollision(tb, a, ta);
            if (tmp.hasCollision)
            {
                c.hasCollision = true;
                if (c.depth < tmp.depth)
                {
                    c.depth = tmp.depth;
                    c.normal = -tmp.normal;
                }
                c.points.insert(c.points.begin(), tmp.points.begin(), tmp.points.end());
                c.pointCount += tmp.pointCount;
            }
        }
        if (flipped)
            c.normal = -c.normal;
        return c;
	}
}