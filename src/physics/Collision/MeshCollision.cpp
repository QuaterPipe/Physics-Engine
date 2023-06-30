#include "physics/Collision/Algo.hpp"
#include <iostream>
#define MAX std::numeric_limits<f64>::max()
#define MIN std::numeric_limits<f64>::min()

namespace physics::algo
{
    CollisionPoints MeshMeshCollision(
		const MeshCollider* a, const Transform& ta,
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