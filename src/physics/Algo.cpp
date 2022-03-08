#include "../include/physics/Algo.hpp"
#include "../include/physics/OstreamOverloads.hpp"
#include <iostream>

namespace algo
{
	CollisionPoints FindCircleCircleCollisionPoints(
		const CircleCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		c.hasCollision = false;
		if (!a || !b) {return c;}
		geometry::Vector ACenter = ta.TransformVector(a->center);
		geometry::Vector BCenter = tb.TransformVector(b->center);
		f64 r = geometry::DistanceSquared(ta.TransformVector(ACenter), tb.TransformVector(BCenter));
		// If the sum of their radii is greater than or equal to the distance between their centers
		if (pow(a->radius + b->radius, 2) >= r)
		{
			geometry::Line l(ta.TransformVector(ACenter), tb.TransformVector(BCenter));
			c.a = l.GetVectorAlongLine(a->radius);
			c.b = l.GetVectorAlongLine(b->radius, false);
			c.depth = geometry::Distance(c.a, c.b);
			c.normal = c.b - c.a;
			c.normal.Normalize();
			c.hasCollision = true;
		}
		return c;
	}

	CollisionPoints FindCircleBoxCollisionPoints(
		const CircleCollider* a, const Transform& ta,
		const BoxCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		if (!a || !b ) {return c;}
		//easier to find collision points as a PolygonCollider
		PolygonCollider* bb = new PolygonCollider(b->pos, b->pos,
			geometry::Vector(b->x + b->width, b->y), 
			geometry::Vector(b->x + b->width, b->y + b->height), {geometry::Vector(b->x , b->y + b->height)});
		return FindPolygonCircleCollisionPoints(bb, tb, a, ta);
	}

	CollisionPoints FindCircleMeshCollisionPoints(
		const CircleCollider* a, const Transform& ta,
		const MeshCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		c.hasCollision = false;
		if (!a || !b) {return c;}
		for (const Collider* ptr: b->colliders)
		{
			c = ptr->TestCollision(tb, a, ta);
			if (c.hasCollision) {return c;}
		}
		return c;
	}

	CollisionPoints FindPolygonCircleCollisionPoints(
		const PolygonCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		c.hasCollision = false;
		if (!a || !b) {return c;}
		if (a->points.size() < 3) {return c;}
		c = CircleInsidePolygon(a, ta, b, tb);
		if (c.hasCollision)
			return c;
		std::vector<geometry::Vector> APoints;
		for (const geometry::Vector& v: a->points)
		{
			APoints.push_back(ta.TransformVector(v + a->pos));
		}
		geometry::Vector BCenter = tb.TransformVector(b->center);
		std::vector<geometry::Line> lines;
		for (unsigned i = 0; i < APoints.size(); i++)
		{
			lines.push_back(geometry::Line(APoints[i], APoints[(i + 1) % APoints.size()]));
		}
		geometry::Vector closest = geometry::Vector::Infinity;
		geometry::Line closestLine;
		f64 dis = std::numeric_limits<f64>::max();
		// project the circle's center onto every line : the PolygonCollider
		for (const geometry::Line& l: lines)
		{
			geometry::Vector proj = geometry::Vector::Projection(BCenter, l);
			if (geometry::DistanceSquared(proj, BCenter) < dis)
			{
				// if the vector is on the line
				if (l.VectorIsOnLine(proj))
				{
					closest = proj;
					dis = geometry::DistanceSquared(proj, BCenter);
					closestLine = l;
				}
			}
		}
		if (dis == std::numeric_limits<f64>::max())
			return c;
		if (geometry::DistanceSquared(closest, BCenter) > b->radius * b->radius)
			return c;
		auto VectorInCircle = [&] (const geometry::Vector& center, const f64& radius, const geometry::Vector point) {
			f64 sqrDis = pow((center.x - point.x), 2) + pow((center.x - point.y), 2);
			return sqrDis <= radius * radius;
		};
		// if the polygon collider is inside the circle
		{
			geometry::Vector farthest = geometry::Vector::Infinity;
			f64 maxDis = std::numeric_limits<f64>::min();
			bool allInCircle = false;
			for (unsigned i = 0; i < APoints.size(); i++)
			{
				if (!VectorInCircle(BCenter, b->radius, APoints[i]))
					break;
				f64 distance = geometry::DistanceSquared(APoints[i], BCenter);
				if (distance > maxDis)
				{
					farthest = APoints[i];
					maxDis = distance;
				}
				if (i == APoints.size() - 1)
					allInCircle = true;
			}
			if (allInCircle)
			{
				c.a = closest;
				geometry::Line tmp(closest, BCenter);
				c.b = tmp.GetVectorAlongLine(b->radius, false);
				// reversed norm vector to push PolygonCollider out of circle				
				c.normal = c.a - c.b;
				c.depth = geometry::Distance(c.a, c.b);
				c.normal.Normalize();
				c.hasCollision = true;
				return c;
			}
		}
		//if the circle's center is inside the PolygonCollider
		if (PolygonColliderVectorIsColliding(a, ta, BCenter))
		{
			c.a = BCenter;
			c.b = closest;
			// reversed norm vector to push circle out of PolygonCollider
			c.normal = c.a - c.b;
			c.depth = geometry::Distance(c.a, c.b);
			c.normal.Normalize();
			c.hasCollision = true;
			return c;
		}
		// else 
		c.a = closest;
		geometry::Line radius(closest, BCenter);
		c.b = radius.GetVectorAlongLine(b->radius, false);
		c.normal = c.b - c.a;
		c.normal.Normalize();
		c.hasCollision = true;
		return c;
	}

	geometry::Vector getCentroid(std::vector<geometry::Vector> points)
	{
		if (points.size())
		{
			geometry::Vector first = points.at(0);
			geometry::Vector last = points.at(points.size() - 1);
			if (first.x != last.x || first.y != last.y)
			{
				points.push_back(first);
			}
			f64 twiceArea = 0, x = 0, y = 0, f = 0;
			geometry::Vector p1, p2;
			// absolutely no clue what this does, it just works lol
			for (size_t i = 0, j = points.size() - 1; i < points.size(); j=i++)
			{
				p1 = points[i]; p2 = points[j];
				f = (p1.y - first.y) * (p2.x - first.x) - (p2.y - first.y) * (p1.x - first.x);
				twiceArea += f;
				x += (p1.x + p2.x - 2 * first.x) * f;
				y += (p1.y + p2.y - 2 * first.y) * f;
			}
			f = twiceArea * 3;
			return geometry::Vector(x / f + first.x, y / f + first.y);
		}
		else
			return geometry::Vector::Origin;
	}

	CollisionPoints FindPolygonPolygonCollisionPoints(
		const PolygonCollider* a, const Transform& ta,
		const PolygonCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		c.hasCollision = false;
		if (!a || !b) {return c;}
		if (a->points.size() < 3 || b->points.size() < 3) {return c;}
		c = PolygonInsidePolygon(a, ta, b, tb);
		if (c.hasCollision)
			return c;
		std::vector<geometry::Vector> APoints;
		std::vector<geometry::Vector> BPoints;
		for (const geometry::Vector& v: a->points)
		{
			APoints.push_back(ta.TransformVector(v) + a->pos);
		}
		for (const geometry::Vector& v: b->points)
		{
			BPoints.push_back(ta.TransformVector(v) + b->pos);
		}
		//check every point : collider b if it intersects with a
		std::vector<geometry::Vector> pointIntersectsB;
		for (const geometry::Vector& p: BPoints)
		{
			if (PolygonColliderVectorIsColliding(a, ta, p))
			{
				pointIntersectsB.push_back(p);
			}
		}
		if (!pointIntersectsB.size())
			return c;
		geometry::Vector closest = pointIntersectsB.at(0);
		bool reached = false;
		geometry::Vector centroidA = getCentroid(APoints);
		for (const geometry::Vector& p: pointIntersectsB)
		{
			if (!reached)
			{
				reached = true; closest = p; continue;
			}
			if (geometry::DistanceSquared(p, centroidA) < geometry::DistanceSquared(closest, centroidA))
				closest = p; 
		}
		c.b = closest;
		std::vector<geometry::Vector> pointIntersectsA;
		for (const geometry::Vector& p: APoints)
		{
			if (PolygonColliderVectorIsColliding(b, tb, p))
			{
				pointIntersectsA.push_back(p );
			}
		}
		if (!pointIntersectsA.size())
			return c;
		closest = geometry::Vector::Origin;
		reached = false;
		geometry::Vector centroidB = getCentroid(BPoints);
		for (const geometry::Vector& p: pointIntersectsA)
		{
			if (!reached)
			{
				reached = true; closest = p; continue;
			}
			if (geometry::DistanceSquared(p, centroidB) < geometry::DistanceSquared(closest, centroidB))
			{
				closest = p;
			}
		}
		c.a = closest;
		c.depth = geometry::Distance(c.a, c.b);
		c.normal = (c.b - c.a);
		c.normal.Normalize();
		c.hasCollision = true;
		return c;
	}

	CollisionPoints FindPolygonBoxCollisionPoints(
		const PolygonCollider* a, const Transform& ta,
		const BoxCollider* b, const Transform& tb)
	{
		CollisionPoints c;
		if (!a || !b) {return c;}
		PolygonCollider* bb = new PolygonCollider(b->pos, b->pos,
			geometry::Vector(b->x + b->width, b->y), 
			geometry::Vector(b->x + b->width, b->y + b->height), {geometry::Vector(b->x , b->y + b->height)});
		return FindPolygonPolygonCollisionPoints(a, ta, bb, tb);
	}

	CollisionPoints FindPolygonMeshCollisionPoints(
		const PolygonCollider* a, const Transform& ta,
		const MeshCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		if (!a || !b) {return c;}
		for (const Collider* ptr: b->colliders)
		{
			c = ptr->TestCollision(tb, a, ta);
			if (c.hasCollision) {return c;}
		}
		return c;
	}

	CollisionPoints FindBoxBoxCollisionPoints(
		const BoxCollider* a, const Transform& ta,
		const BoxCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		if (!a || !b) {return c;}
		PolygonCollider* bb = new PolygonCollider(b->pos, b->pos,
			geometry::Vector(b->x + b->width, b->y), 
			geometry::Vector(b->x + b->width, b->y + b->height), {geometry::Vector(b->x , b->y + b->height)});
		PolygonCollider* aa = new PolygonCollider(a->pos, a->pos,
			geometry::Vector(a->x + a->width, a->y), 
			geometry::Vector(a->x + a->width, a->y + a->height), {geometry::Vector(a->x , a->y + a->height)});
		return FindPolygonPolygonCollisionPoints(aa, ta, bb, tb);
	}

	CollisionPoints FindBoxMeshCollisionPoints(
		const BoxCollider* a, const Transform& ta,
		const MeshCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		if (!a || !b) {return c;}
		for (const Collider* ptr: b->colliders)
		{
			c = ptr->TestCollision(tb, a, ta);
			if (c.hasCollision) {return c;}
		}
		return c;
	}

	CollisionPoints FindMeshMeshCollisionPoints(
		const MeshCollider* a, const Transform& ta,
		const MeshCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		if (!a || !b) {return c;}
		for (const Collider* ptrA: a->colliders)
		{
			for (const Collider* ptrB: a->colliders)
			{
				c = ptrA->TestCollision(ta, ptrB, tb);
				if (c.hasCollision) {return c;}
			}
		}
		return c;
	}

	bool LinePassesThroughCircle(
		const geometry::Line& a, const CircleCollider* b,
		const Transform& tb
	)
	{
		if (geometry::DistanceSquared(a.a, b->center + tb.position) <= b->radius * b->radius)
			return true;
		else if (geometry::DistanceSquared(a.b, b->center + tb.position) <= b->radius * b->radius)
			return true;
		geometry::Vector v = geometry::Vector::Projection(b->center + tb.position, a);
		return geometry::DistanceSquared(v, b->center + tb.position) <= b->radius * b->radius;
	}

	bool CircleInsideCircle(
		const CircleCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	)
	{
		return geometry::DistanceSquared(a->center + ta.position, 
			b->center + tb.position) < fabs(a->radius * a->radius - b->radius * b->radius);
	}

	CollisionPoints CircleInsidePolygon(
		const PolygonCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		c.hasCollision = false;
		if (!a || !b) return c;
		if (a->points.size() < 3) return c;
		if (!PolygonColliderVectorIsColliding(a, ta, tb.TransformVector(b->center)))
			return c;
		std::vector<geometry::Vector> APoints;
		for (const geometry::Vector& v: a->points)
		{
			APoints.push_back(ta.TransformVector(v) + a->pos);
		}
		geometry::Vector BCenter = tb.TransformVector(b->center);
		geometry::Vector projections[APoints.size()];
		geometry::Vector closest;
		f64 distance = std::numeric_limits<f64>::infinity();
		for (size_t i = 0; i < APoints.size(); i++)
		{
			geometry::Line l(APoints[i], APoints[(i + 1) % APoints.size()]);
			projections[i] = geometry::Vector::Projection(BCenter, l);
			if (geometry::DistanceSquared(projections[i], BCenter) < b->radius * b->radius)
				return c;
			if (geometry::DistanceSquared(projections[i], BCenter) < distance)
			{
				closest = projections[i];
				distance = geometry::DistanceSquared(closest, BCenter);
			}
		}
		c.b = closest;
		c.a = geometry::Line(BCenter, closest).GetVectorAlongLine(b->radius);
		c.normal = c.a - c.b;
		c.normal.Normalize();
		c.hasCollision = true;
		c.depth = geometry::Distance(c.a, c.b);
		return c;
	}

	CollisionPoints PolygonInsidePolygon(
		const PolygonCollider* a, const Transform& ta,
		const PolygonCollider* b, const Transform& tb
	)
	{
		// farthest point from A's centroid
		if (!a || !b) {return CollisionPoints();}
		std::vector<geometry::Vector> APoints;
		std::vector<geometry::Vector> BPoints;
		for (const geometry::Vector& v: a->points)
		{
			APoints.push_back(ta.TransformVector(v) + a->pos);
		}
		for (const geometry::Vector& v: b->points)
		{
			BPoints.push_back(tb.TransformVector(v) + b->pos);
		}
		geometry::Vector farthest;
		geometry::Vector centroidA = getCentroid(APoints);
		f64 distance = std::numeric_limits<f64>::min();
		bool allInPolygon = false;
		for (size_t i = 0; i < APoints.size(); i++)
		{
			if (!PolygonColliderVectorIsColliding(b, tb, APoints[i]))
				break;
			if (geometry::DistanceSquared(APoints[i], centroidA) > distance)
			{
				farthest = APoints[i];
				distance = geometry::DistanceSquared(APoints[i], centroidA);
			}
			if (i == APoints.size() - 1)
				allInPolygon = true;
		}
		if (allInPolygon)
		{
			geometry::Vector closest = geometry::Vector::Infinity;
			f64 dis = closest.x; // infinity
			for (size_t i = 0; i < BPoints.size(); i++)
			{
				geometry::Line l(BPoints[i], BPoints[(i + 1) % BPoints.size()]);
				auto tmp = geometry::Vector::Projection(farthest, l);
				if (geometry::DistanceSquared(tmp, farthest) < dis)
				{
					closest = tmp;
					dis = geometry::DistanceSquared(tmp, farthest);
				}
			}
			CollisionPoints c;
			c.a = farthest;
			c.b = closest;
			// pushing collider a out of collider b
			c.normal = c.a - c.b;
			c.normal.Normalize();
			c.hasCollision = true;
			c.depth = geometry::Distance(c.a, c.b);
			return c;
		}
		else
		{
			distance = std::numeric_limits<f64>::min();
			// do not need to reset allInPolygon since it is already false
			geometry::Vector centroidB = getCentroid(BPoints);
			for (size_t i = 0; i < BPoints.size(); i++)
			{
				if (!PolygonColliderVectorIsColliding(a, ta, BPoints[i]))
					break;
				if (geometry::DistanceSquared(BPoints[i], centroidB) > distance)
				{
					farthest = BPoints[i];
					distance = geometry::DistanceSquared(BPoints[i], centroidB);
				}
				if (i == BPoints.size() - 1)
					allInPolygon = true;
			}
			if (allInPolygon)
			{
				geometry::Vector closest = geometry::Vector::Infinity;
				f64 dis = closest.x; // infinity
				for (size_t i = 0; i < APoints.size(); i++)
				{
					geometry::Line l(APoints[i], APoints[(i + 1) % APoints.size()]);
					auto tmp = geometry::Vector::Projection(farthest, l);
					if (geometry::DistanceSquared(tmp, farthest) < dis)
					{
						closest = tmp;
						dis = geometry::DistanceSquared(tmp, farthest);
					}
				}
				CollisionPoints c;
				c.b = farthest;
				c.a = closest;
				// pushing collider a out of collider b
				c.normal = c.b - c.a;
				c.normal.Normalize();
				c.hasCollision = true;
				c.depth = geometry::Distance(c.b, c.a);
				return c;
			}
		}
		return CollisionPoints();
	}

	bool PolygonColliderVectorIsColliding(
		const PolygonCollider* a, const Transform& ta,
		const geometry::Vector& b)
	{
		/*
		* All this does is draw a (horizontal)line from b to the farthest point : a, if the amount
		* of intersections with the polygon is even, b is not inside if it is odd b is inside.
		*/
		if (a->points.size() <3)/* ily too <3*/ {return false;}
		std::vector<geometry::Vector> APoints;
		for (const geometry::Vector& v: a->points)
		{
			APoints.push_back(ta.TransformVector(v) + a->pos);
		}
		geometry::Vector max = (*max_element(APoints.begin(), APoints.end()));
		geometry::Line line(b, geometry::Vector(max.x + (1 * max.x > b.x ? 1 : -1), b.y));
		std::vector<geometry::Vector> listOfIntersections = std::vector<geometry::Vector>();
		for (size_t i = 0; i < APoints.size(); i++)
		{
			geometry::Line l(APoints[i], APoints[(i + 1) % APoints.size()]);
			if (geometry::Intersecting(l, line))
			{
				if (!listOfIntersections.size() || !std::count(listOfIntersections.begin(), listOfIntersections.end(), (geometry::VectorOfIntersect(l, line))))
					listOfIntersections.push_back(geometry::VectorOfIntersect(l, line));
			}
		}
		return listOfIntersections.size() % 2;
	}
}