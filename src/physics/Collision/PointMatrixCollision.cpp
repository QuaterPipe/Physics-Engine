#include "physics/Collision/Algo.hpp"
#include "physics/Collision/PointMatrixCollider.hpp"
#include <iostream>

namespace physics::algo
{
	Vector3 SATSE(Vector2* aPoints, size_t aSize, Vector2* bPoints, size_t bSize) noexcept
	{
		f64 minOverlap = std::numeric_limits<f64>::max();
		size_t minInd = 0;
		Vector2 smallestAxis;
		Vector2* edges = new Vector2[bSize];
		for (size_t i = 0; i < bSize; i++)
		{
			edges[i] = bPoints[(i + 1) % bSize] - bPoints[i];
			edges[i].Set(-edges[i].y, edges[i].x);
			edges[i].Normalize();
		}
		for (size_t i = 0; i < bSize; i++)
		{
			if (edges[i].GetMagnitudeSquared() <= SQRD(EPSILON))
				continue;
			f64 minA = std::numeric_limits<f64>::max(), maxA = std::numeric_limits<f64>::min();
			for (size_t j = 0; j < aSize; j++)
			{
				f64 p = aPoints[j].Dot(edges[i]);
				minA = Min(minA, p);
				maxA = Max(maxA, p);
			}

			f64 minB = std::numeric_limits<f64>::max(), maxB = std::numeric_limits<f64>::min();
			for (size_t j = 0; j < bSize; j++)
			{
				f64 p = bPoints[j].Dot(edges[i]);
				minB = Min(minB, p);
				maxB = Max(maxB, p);
			}

			if (maxA < minB || maxB < minA)
			{
				delete[] edges;
				return Vector3::Infinity;
			}
			f64 overlap = Min(maxB - minA, maxA - minB);
			if (overlap < minOverlap)
			{
				minOverlap = overlap;
				smallestAxis = edges[i];
				minInd = i;
				f64 centerA = 0.5 * (minA + maxA);
				f64 centerB = 0.5 * (minB + maxB);
				if (centerB < centerA)
					smallestAxis *= -1;
			}
		}
		delete[] edges;
		smallestAxis *= minOverlap;
		Vector3 result(smallestAxis.x, smallestAxis.y, minInd);
		return result;
	}

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

		size_t aSize = a->GetPointCount();
		Vector2* aPoints = new Vector2[aSize];
		for (size_t i = 0; i < aSize; i++)
			aPoints[i] = ta.TransformVector(a->GetPoint(i));

		size_t bSize = b->GetPointCount();
		Vector2* bPoints = new Vector2[bSize];
		for (size_t i = 0; i < bSize; i++)
			bPoints[i] = tb.TransformVector(b->GetPoint(i));

		for (size_t i = 0; i < aSize; i++)
		{
			if (VectorInPolygon(bPoints, aPoints[i], bSize))
			{
				c.points.push_back(aPoints[i]);
				c.hasCollision = true;
			}
		}
		if (!c.hasCollision)
		{
			delete[] aPoints;
			delete[] bPoints;
			return c;
		}
		Vector3 vec = SATSE(aPoints, aSize, bPoints, bSize);
		delete[] aPoints;
		delete[] bPoints;
		Vector2 axis(vec.x, vec.y);
		c.depth = axis.GetMagnitude();
		c.normal = axis / c.depth;
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
		if (!a->edgeCount || !b->edgeCount) // temporary
			return c;
		size_t aSize = a->GetPointCount();
		Vector2* aPoints = new Vector2[aSize];
		size_t bSize = b->GetPointCount();
		Vector2* bPoints = new Vector2[bSize];
		for (size_t i = 0; i < aSize; i++)
			aPoints[i] = ta.TransformVector(a->GetPoint(i));
		for (size_t i = 0; i < bSize; i++)
			bPoints[i] = tb.TransformVector(b->GetPoint(i));
		size_t aEdgeCount = a->edgeCount;
		Vector2* aPolyPoints = new Vector2[aEdgeCount];
		size_t bEdgeCount = b->edgeCount;
		Vector2* bPolyPoints = new Vector2[bEdgeCount];
		for (size_t i = 0; i < aEdgeCount; i++)
			aPolyPoints[i] = aPoints[a->edges[i]];
		for (size_t i = 0; i < bEdgeCount; i++)
			bPolyPoints[i] = bPoints[b->edges[i]];
		Vector3 check = SAT(aPolyPoints, aEdgeCount, bPolyPoints, bEdgeCount);
		if (check == Vector3::Infinity)
		{
			delete[] aPoints;
			delete[] bPoints;
			delete[] aPolyPoints;
			delete[] bPolyPoints;
			return c;
		}
		Vector2 axis(check.x, check.y);

		for (size_t i = 0; i < aSize; i++)
			if (VectorInPolygon(bPolyPoints, aPoints[i], bEdgeCount))
				c.points.push_back(aPoints[i]);
		for (size_t i = 0; i < bSize; i++)
			if (VectorInPolygon(aPolyPoints, bPoints[i], aEdgeCount))
				c.points.push_back(bPoints[i]);
		delete[] aPoints;
		delete[] bPoints;
		delete[] aPolyPoints;
		delete[] bPolyPoints;
		c.pointCount = c.points.size();
		c.depth = axis.GetMagnitudeExact();
		c.normal = axis / c.depth;
		c.hasCollision = true;
		if (flipped)
			c.normal = -c.normal;
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