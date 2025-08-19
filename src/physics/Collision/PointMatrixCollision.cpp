#include "physics/Collision/Algo.hpp"
#include "physics/Collision/PointMatrixCollider.hpp"

namespace physics::algo
{
	Manifold PointMatrixCircleCollision(
		const PointMatrixCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb, bool flipped
	)
	{
		Manifold c;
		c.hasCollision = false;
		if (!a || !b)
			return c;
		if (a->GetPointCount() < 3)
			return c;
		size_t aSize = a->GetPointCount();
		Vector2* aPoints = new Vector2[aSize];
		for (size_t i = 0; i < aSize; i++)
			aPoints[i] = ta.TransformVector(a->GetPoint(i));
		Vector2 bCenter = tb.TransformVector(b->center);
		f64 bRadius = b->radius * Max(tb.GetScale().x, tb.GetScale().y);
		Vector2 deepest = Vector2::Infinity;
		f64 minDis = std::numeric_limits<f64>::infinity();
		for (size_t i = 0; i < aSize; i++)
		{
			f64 dis = DistanceSquared(aPoints[i], bCenter);
			if (dis > bRadius)
				continue;
			c.points.push_back(aPoints[i]);
			if (dis < minDis)
			{
				minDis = dis;
				deepest = aPoints[i];
			}
		}
		delete[] aPoints;
		if (!c.points.size())
			return c;
		c.pointCount = c.points.size();
		c.depth = bRadius - sqrt(minDis);
		c.hasCollision = true;
		c.normal = (deepest - bCenter) / c.depth;
		if (flipped)
			c.normal *= -1;
		return c;
	}

	Manifold PointMatrixBoxCollision(
		const PointMatrixCollider* a, const Transform& ta,
		const BoxCollider* b, const Transform& tb, bool flipped
	)
	{
		Manifold c;
		c.hasCollision = false;
		if (!a || !b)
			return c;
		PolygonCollider pCol = PolygonCollider(*b);
		return PointMatrixPolygonCollision(a, ta, &pCol, tb, flipped);
	}

	Manifold PointMatrixPolygonCollision(
		const PointMatrixCollider* a, const Transform& ta,
		const PolygonCollider* b, const Transform& tb, bool flipped
	)
	{
		Manifold c;
		c.hasCollision = false;
		if (!a || !b)
			return c;
		if (b->GetPointCount() < 3)
			return c;
		Vector2 bCenter = tb.TransformVector(b->GetCenter());
		Vector2* aPoints = new Vector2[a->GetPointCount()];
		size_t aSize = a->GetPointCount();
		for (size_t i = 0; i < aSize; i++)
			aPoints[i] = ta.TransformVector(a->GetPoint(i));
		Vector2* bPoints = new Vector2[b->GetPointCount()];
		size_t bSize = b->GetPointCount();
		for (size_t i = 0; i < bSize; i++)
			bPoints[i] = tb.TransformVector(b->GetPoint(i));
		Vector2 closestPoint;
		f64 closestDis = std::numeric_limits<f64>::max();
		for (size_t i = 0; i < aSize; i++)
		{
			if (VectorInPolygon(bPoints, aPoints[i], bSize));
			{
				c.points.push_back(aPoints[i]);
				c.hasCollision = true;
				f64 dis = DistanceSquared(aPoints[i], bCenter);
				if (dis < closestDis)
				{
					closestDis = dis;
					closestPoint = aPoints[i];
				}
			}
		}
		if (!c.hasCollision)
		{
			delete[] aPoints;
			delete[] bPoints;
			return c;
		}
		f64 minDis = std::numeric_limits<f64>::max();
		Vector2 dir;
		for (size_t i = 0; i < bSize; i++)
		{
			Line l(bPoints[i], bPoints[(i + 1) % bSize]);
			f64 dis = Distance(l, closestPoint);
			if (dis < minDis)
			{
				minDis = dis;
				Vector2 v(bPoints[i] - bPoints[(i + 1) % bSize]);
				v.Set(-v.y, v.x);
				dir = v;
			}
		}
		c.depth = minDis;
		c.normal = dir.Normalized();
		if (flipped)
			c.normal = -c.normal;
		c.pointCount = c.points.size();
		return c;
	}

	Manifold PointMatrixPointMatrixCollision(
		const PointMatrixCollider* a, const Transform& ta,
		const PointMatrixCollider* b, const Transform& tb, bool flipped
	)
	{
		Manifold c;
		if (!a || !b)
			return c;
		if (a->GetPointCount() < 4 || b->GetPointCount() < 4)
			return c;
		size_t aSize = a->GetPointCount();
		Vector2* aPoints = new Vector2[aSize];
		size_t bSize = b->GetPointCount();
		Vector2* bPoints = new Vector2[bSize];
		for (size_t i = 0; i < aSize; i++)
			aPoints[i] = ta.TransformVector(a->GetPoint(i));
		for (size_t i = 0; i < bSize; i++)
			bPoints[i] = tb.TransformVector(b->GetPoint(i));
		Vector2* aOutline = new Vector2[1];

		return c;
	}

	Manifold PointMatrixMeshCollision(
		const PointMatrixCollider* a, const Transform& ta,
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