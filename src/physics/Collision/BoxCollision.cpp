#include "../../include/physics/Collision/Algo.hpp"
#include <iostream>
#define MAX std::numeric_limits<f64>::max()
#define MIN std::numeric_limits<f64>::min()

namespace physics::algo
{
    CollisionPoints BoxBoxCollision(
		const BoxCollider* a, const Transform& ta,
		const BoxCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		if (!a || !b) {return c;}
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
		if (!a || !b) {return c;}
		for (const Collider* ptr: b->colliders)
		{
			c = ptr->TestCollision(tb, a, ta);
			if (c.hasCollision)
				return c;
		}
		return c;
	}
}