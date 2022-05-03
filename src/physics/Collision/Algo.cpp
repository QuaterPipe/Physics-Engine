#include "../../include/physics/Collision/Algo.hpp"
#include "../../include/physics/Tools/OstreamOverloads.hpp"
#include <iostream>

namespace physics::algo
{

	CollisionPoints PointCircleCollision(
		const PointCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		c.hasCollision = false;
		if (!a || !b) {return c;}
		const geo::Vector BCenter = tb.TransformVector(b->center);
		const geo::Vector APos = ta.TransformVector(a->position);
		if (geo::DistanceSquared(BCenter, APos) <= SQRD(b->radius))
		{
			c.a = APos;
			geo::Vector tmp = (APos - BCenter).Normalized();
			c.b = tmp * b->radius + BCenter;
			c.depth = geo::Distance(c.a, c.b);
			c.normal = (c.b - c.a).Normalized();
			c.hasCollision = true;
		}
		return c;
	}

	CollisionPoints PointPolygonCollision(
		const PointCollider* a, const Transform& ta,
		const PolygonCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		c.hasCollision = false;
		if (!a || !b) return c;
		geo::Vector APos = ta.TransformVector(a->position);
		if (!PolygonColliderVectorIsColliding(b, tb, APos))
			return c;
		std::vector<geo::Vector> BPoints;
		for (const geo::Vector& v: b->points)
		{
			BPoints.push_back(tb.TransformVector(v + b->pos));
		}
		c.b = APos;
		geo::Vector closest = geo::Vector::Infinity;
		for (size_t i = 0; i < BPoints.size(); i++)
		{
			geo::Line l(BPoints[i], BPoints[(i + 1) % BPoints.size()]);
			geo::Vector p = geo::Vector::Projection(APos, l);
			if (geo::DistanceSquared(closest, APos) > geo::DistanceSquared(p, APos))
			{
				if (l.VectorIsOnLine(p))
					closest = p;
			}
		}
		c.a = closest;
		c.depth = geo::Distance(c.a, c.b);
		c.normal = (c.b - c.a).Normalized();
		c.hasCollision = true;
		return c;
	}

	CollisionPoints PointBoxCollision(
		const PointCollider* a, const Transform& ta,
		const BoxCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		c.hasCollision = false;
		if (!a || !b) return c;
		PolygonCollider* bb = new PolygonCollider(b->pos, b->pos,
			geo::Vector(b->x + b->width, b->y), 
			geo::Vector(b->x + b->width, b->y + b->height), {geo::Vector(b->x , b->y + b->height)});
		return PointPolygonCollision(a, ta, bb, tb);
	}

	CollisionPoints PointMeshCollision(
		const PointCollider* a, const Transform& ta,
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

	CollisionPoints PointPointCollision(
		const PointCollider* a, const Transform& ta,
		const PointCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		c.hasCollision = false;
		if (!a || !b) {return c;}
		if (geo::DistanceSquared(ta.TransformVector(a->position), tb.TransformVector(b->position)) < SQRD(0.000001))
		{
			c.a = a->position;
			c.b = b->position;
			c.depth = geo::Distance(c.a, c.b);
			c.normal = (c.b - c.a).Normalized();
			c.hasCollision = true;
		}
		return c;
	}

	CollisionPoints CircleCircleCollision(
		const CircleCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		c.hasCollision = false;
		if (!a || !b) {return c;}
		const geo::Vector ACenter = ta.TransformVector(a->center);
		const geo::Vector BCenter = tb.TransformVector(b->center);
		f64 r = geo::DistanceSquared(ACenter, BCenter);
		// If the sum of their radii is greater than or equal to the distance between their centers
		if (SQRD(a->radius + b->radius) >= r)
		{
			geo::Line l(ACenter, BCenter);
			c.a = l.GetVectorAlongLine(a->radius);
			c.b = l.GetVectorAlongLine(b->radius, false);
			c.depth = geo::Distance(c.a, c.b);
			c.normal = c.b - c.a;
			c.normal.Normalize();
			c.hasCollision = true;
		}
		return c;
	}

	CollisionPoints CircleBoxCollision(
		const CircleCollider* a, const Transform& ta,
		const BoxCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		if (!a || !b ) {return c;}
		//easier to  collision points as a PolygonCollider
		PolygonCollider* bb = new PolygonCollider(b->pos, b->pos,
			geo::Vector(b->x + b->width, b->y), 
			geo::Vector(b->x + b->width, b->y + b->height), {geo::Vector(b->x , b->y + b->height)});
		return PolygonCircleCollision(bb, tb, a, ta);
	}

	CollisionPoints CircleMeshCollision(
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

	CollisionPoints PolygonCircleCollision(
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
		std::vector<geo::Vector> APoints;
		for (const geo::Vector& v: a->points)
			APoints.push_back(ta.TransformVector(v + a->pos));
		
		const geo::Vector BCenter = tb.TransformVector(b->center);
		std::vector<geo::Line> lines;
		for (unsigned i = 0; i < APoints.size(); i++)
			lines.push_back(geo::Line(APoints[i], APoints[(i + 1) % APoints.size()]));
		
		geo::Vector closest = geo::Vector::Infinity;
		geo::Line closestLine;
		f64 dis = std::numeric_limits<f64>::max();
		// project the circle's center onto every line : the PolygonCollider
		for (const geo::Line& l: lines)
		{
			geo::Vector proj = geo::Vector::Projection(BCenter, l);
			if (geo::DistanceSquared(proj, BCenter) < dis && l.VectorIsOnLine(proj))
			{  
				closest = proj;
				dis = geo::DistanceSquared(proj, BCenter);
				closestLine = l;
			}
		}
		auto VectorInCircle = [&] (const geo::Vector& center, const f64& radius, const geo::Vector point) {
			return geo::DistanceSquared(center, point) <= SQRD(radius);
		};
		// if the polygon collider is completely inside the circle
		{
			geo::Vector farthest = geo::Vector::Infinity;
			f64 maxDis = std::numeric_limits<f64>::min();
			bool allInCircle = true;
			for (geo::Vector& v: APoints)
			{
				if (!VectorInCircle(BCenter, b->radius, v))
				{
					allInCircle = false;
					break;
				}
				f64 distance = geo::DistanceSquared(v, BCenter);
				if (distance > maxDis)
				{
					farthest = v;
					maxDis = distance;
				}
			}
			if (allInCircle)
			{
				c.a = farthest;
				geo::Line tmp(farthest, BCenter);
				c.b = tmp.GetVectorAlongLine(b->radius, false);
				// reversed norm vector to push PolygonCollider out of circle				
				c.normal = c.a - c.b;
				c.depth = geo::Distance(c.a, c.b);
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
			c.depth = geo::Distance(c.a, c.b);
			c.normal.Normalize();
			c.hasCollision = true;
			return c;
		}
		// else 
		c.a = closest;
		geo::Line radius(closest, BCenter);
		c.b = radius.GetVectorAlongLine(b->radius, false);
		c.normal = c.b - c.a;
		c.normal.Normalize();
		c.hasCollision = true;
		return c;
	}

	CollisionPoints PolygonPolygonCollision(
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
		std::vector<geo::Vector> APoints;
		std::vector<geo::Vector> BPoints;
		for (const geo::Vector& v: a->points)
			APoints.push_back(ta.TransformVector(v + a->pos));
		
		for (const geo::Vector& v: b->points)
			BPoints.push_back(tb.TransformVector(v + b->pos));
		
		bool BPointsInA = false;
		for (const geo::Vector& p: BPoints)
		{
			if (PolygonColliderVectorIsColliding(a, ta, p))
				BPointsInA = true;
		}
		bool APointsInB = false;
		for (const geo::Vector& p: APoints)
		{
			if (PolygonColliderVectorIsColliding(b, tb, p))
				APointsInB = true;
		}
		auto intersections = GetIntersectionsBetweenTwoPolygons(a, ta, b, tb);
		if (!intersections.size())
			return c;
		if (BPointsInA)
		{
			geo::Vector closest;
			f64 distance = 0;
			bool reached = false;
			geo::Vector centroid = geo::Centroid(&*APoints.begin(), &*APoints.end());
			for (geo::Vector& v: BPoints)
			{
				if (!reached)
				{
					reached = true;
					closest = v;
					distance = geo::DistanceSquared(centroid, v);
				}
				if (distance > geo::DistanceSquared(centroid, v))
				{
					closest = v;
					distance = geo::DistanceSquared(centroid, closest);
				}
			}
			c.b = closest;
		}
		else
		{
			geo::Vector closest;
			f64 distance = 0;
			bool reached = false;
			geo::Vector centroid = geo::Centroid(&*APoints.begin(), &*APoints.end());
			for (geo::Vector& v: intersections)
			{
				if (!reached)
				{
					reached = true;
					closest = v;
					distance = geo::DistanceSquared(centroid, v);
				}
				if (distance > geo::DistanceSquared(centroid, v))
				{
					closest = v;
					distance = geo::DistanceSquared(centroid, closest);
				}
			}
			c.b = closest;
		}
		if (APointsInB)
		{
			geo::Vector closest;
			f64 distance = 0;
			bool reached = false;
			geo::Vector centroid = geo::Centroid(&*BPoints.begin(), &*BPoints.end());
			for (geo::Vector& v: APoints)
			{
				if (!reached)
				{
					reached = true;
					closest = v;
					distance = geo::DistanceSquared(centroid, v);
				}
				if (distance > geo::DistanceSquared(centroid, v))
				{
					closest = v;
					distance = geo::DistanceSquared(centroid, closest);
				}
			}
			c.a = closest;
		}
		else
		{
			geo::Vector closest;
			f64 distance = 0;
			bool reached = false;
			geo::Vector centroid = geo::Centroid(&*BPoints.begin(), &*BPoints.end());
			for (geo::Vector& v: intersections)
			{
				if (!reached)
				{
					reached = true;
					closest = v;
					distance = geo::DistanceSquared(centroid, v);
				}
				if (distance > geo::DistanceSquared(centroid, v))
				{
					closest = v;
					distance = geo::DistanceSquared(centroid, closest);
				}
			}
			c.a = closest;
		}
		c.depth = geo::Distance(c.a, c.b);
		c.normal = c.b - c.a;
		c.normal.Normalize();
		c.hasCollision = true;
		return c;
	}

	CollisionPoints PolygonBoxCollision(
		const PolygonCollider* a, const Transform& ta,
		const BoxCollider* b, const Transform& tb)
	{
		CollisionPoints c;
		if (!a || !b) {return c;}
		PolygonCollider* bb = new PolygonCollider(b->pos, b->pos,
			geo::Vector(b->x + b->width, b->y), 
			geo::Vector(b->x + b->width, b->y + b->height), {geo::Vector(b->x , b->y + b->height)});
		return PolygonPolygonCollision(a, ta, bb, tb);
	}

	CollisionPoints PolygonMeshCollision(
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

	CollisionPoints BoxBoxCollision(
		const BoxCollider* a, const Transform& ta,
		const BoxCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		if (!a || !b) {return c;}
		PolygonCollider* bb = new PolygonCollider(b->pos, b->pos,
			geo::Vector(b->x + b->width, b->y), 
			geo::Vector(b->x + b->width, b->y + b->height), {geo::Vector(b->x , b->y + b->height)});
		PolygonCollider* aa = new PolygonCollider(a->pos, a->pos,
			geo::Vector(a->x + a->width, a->y), 
			geo::Vector(a->x + a->width, a->y + a->height), {geo::Vector(a->x , a->y + a->height)});
		return PolygonPolygonCollision(aa, ta, bb, tb);
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
			if (c.hasCollision) {return c;}
		}
		return c;
	}

	CollisionPoints MeshMeshCollision(
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
		const geo::Line& a, const CircleCollider* b,
		const Transform& tb
	)
	{
		if (geo::DistanceSquared(a.a, b->center + tb.position) <= b->radius * b->radius)
			return true;
		else if (geo::DistanceSquared(a.b, b->center + tb.position) <= b->radius * b->radius)
			return true;
		geo::Vector v = geo::Vector::Projection(b->center + tb.position, a);
		return geo::DistanceSquared(v, b->center + tb.position) <= b->radius * b->radius;
	}

	bool CircleInsideCircle(
		const CircleCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	)
	{
		return geo::DistanceSquared(a->center + ta.position, 
			b->center + tb.position) < fabs(a->radius * a->radius - b->radius * b->radius);
	}


	// TODO: optimise this garbage
	CollisionPoints CircleInsidePolygon(
		const PolygonCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		c.hasCollision = false;
		if (!a || !b) return c;
		if (a->points.size() < 3) return c;
		std::vector<geo::Vector> APoints;
		const geo::Vector BCenter = tb.TransformVector(b->center);
		for (const geo::Vector& v: a->points)
		{
			if (geo::DistanceSquared(ta.TransformVector(v + a->pos), BCenter) > SQRD(b->radius))
				return c;
			APoints.push_back(ta.TransformVector(v + a->pos));
		}
		geo::Vector closest = geo::Vector::Infinity;
		f64 distance = std::numeric_limits<f64>::infinity();
		for (size_t i = 0; i < APoints.size(); i++)
		{
			geo::Line l(APoints[i], APoints[(i + 1) % APoints.size()]);
			geo::Vector proj = geo::Vector::Projection(BCenter, l);
			if (!l.VectorIsOnLine(proj))
				continue;
			if (geo::DistanceSquared(proj, BCenter) > SQRD(b->radius))
				return c;
			if (geo::DistanceSquared(proj, BCenter) < distance)
			{
				closest = proj;
				distance = geo::DistanceSquared(closest, BCenter);
			}
		}
		if (closest == geo::Vector::Infinity)
		{
			geo::Vector farthest = closest;
			for (geo::Vector v: APoints) {
				if (geo::DistanceSquared(v, BCenter) < geo::DistanceSquared(farthest, BCenter)) {
					farthest = v;
				}
			}
			c.b = farthest;
			c.a = geo::Line(BCenter, farthest).GetVectorAlongLine(b->radius);
			c.normal = c.b - c.a;
			c.normal.Normalize();
			c.hasCollision = true;
			c.depth = geo::Distance(c.a, c.b);
		}
		if (geo::DistanceSquared(closest, BCenter) > SQRD(b->radius))
			return c;
		c.b = closest;
		c.a = geo::Line(BCenter, closest).GetVectorAlongLine(b->radius);
		c.normal = c.b - c.a;
		c.normal.Normalize();
		c.hasCollision = true;
		c.depth = geo::Distance(c.a, c.b);
		return c;
	}

	CollisionPoints PolygonInsidePolygon(
		const PolygonCollider* a, const Transform& ta,
		const PolygonCollider* b, const Transform& tb
	)
	{
		// farthest point from A's centroid
		if (!a || !b) {return CollisionPoints();}
		std::vector<geo::Vector> APoints;
		std::vector<geo::Vector> BPoints;
		for (const geo::Vector& v: a->points)
		{
			APoints.push_back(ta.TransformVector(v + a->pos));
		}
		for (const geo::Vector& v: b->points)
		{
			BPoints.push_back(tb.TransformVector(v + b->pos));
		}
		geo::Vector farthest;
		geo::Vector centroidA = geo::Centroid(&*APoints.begin(),(&*APoints.end()));
		f64 distance = std::numeric_limits<f64>::min();
		// if all of a's points are in b
		bool allInPolygon = false;
		//  checking if all of a's vectors are in b, while getting the farthest point from a's centroid
		for (size_t i = 0; i < APoints.size(); i++)
		{
			// checking if current point is inside collider b
			if (!PolygonColliderVectorIsColliding(b, tb, APoints[i]))
				break;
			if (geo::DistanceSquared(APoints[i], centroidA) > distance)
			{
				farthest = APoints[i];
				distance = geo::DistanceSquared(APoints[i], centroidA);
			}
			if (i == APoints.size() - 1)
				allInPolygon = true;
		}
		// so if a is inside b
		if (allInPolygon)
		{
			geo::Vector closest = geo::Vector::Infinity;
			f64 dis = closest.x; // infinity
			for (size_t i = 0; i < BPoints.size(); i++)
			{
				geo::Line l(BPoints[i], BPoints[(i + 1) % BPoints.size()]);
				auto tmp = geo::Vector::Projection(farthest, l);
				if (geo::DistanceSquared(tmp, farthest) < dis && tmp != farthest)
				{
					closest = tmp;
					dis = geo::DistanceSquared(tmp, farthest);
				}
			}
			CollisionPoints c;
			c.a = farthest;
			c.b = closest;
			// pushing collider a out of collider b
			c.normal = c.a - c.b;
			c.normal.Normalize();
			c.hasCollision = true;
			c.depth = geo::Distance(c.a, c.b);
			return c;
		}
		else
		{
			distance = std::numeric_limits<f64>::min();
			// do not need to reset allInPolygon since it is already false
			geo::Vector centroidB = geo::Centroid(&*BPoints.begin(), &*BPoints.end());
			for (size_t i = 0; i < BPoints.size(); i++)
			{
				if (!PolygonColliderVectorIsColliding(a, ta, BPoints[i]))
					break;
				if (geo::DistanceSquared(BPoints[i], centroidB) > distance)
				{
					farthest = BPoints[i];
					distance = geo::DistanceSquared(BPoints[i], centroidB);
				}
				if (i == BPoints.size() - 1)
					allInPolygon = true;
			}
			if (allInPolygon)
			{
				geo::Vector closest = geo::Vector::Infinity;
				f64 dis = closest.x; // infinity
				for (size_t i = 0; i < APoints.size(); i++)
				{
					geo::Line l(APoints[i], APoints[(i + 1) % APoints.size()]);
					auto tmp = geo::Vector::Projection(farthest, l);
					if (geo::DistanceSquared(tmp, farthest) < dis && farthest != tmp)
					{
						closest = tmp;
						dis = geo::DistanceSquared(tmp, farthest);
					}
				}
				CollisionPoints c;
				c.b = farthest;
				c.a = closest;
				// pushing collider a out of collider b
				c.normal = c.a - c.b;
				c.normal.Normalize();
				c.hasCollision = true;
				c.depth = geo::Distance(c.b, c.a);
				return c;
			}
		}
		return CollisionPoints();
	}

	bool PolygonColliderVectorIsColliding(
		const PolygonCollider* a, const Transform& ta,
		const geo::Vector& b)
	{
		/*
		* All this does is draw a (horizontal)line from b to the farthest point : a, if the amount
		* of intersections with the polygon is even, b is not inside if it is odd b is inside.
		*/
		if (a->points.size() <3)/* ily too <3*/ {return false;}
		std::vector<geo::Vector> APoints;
		for (const geo::Vector& v: a->points)
			APoints.push_back(ta.TransformVector(v + a->pos));
		geo::Vector max = -geo::Vector::Infinity;
		for (geo::Vector v: APoints)
		{
			if (max.x < v.x)
				max = v;
		}
		max.x++;
		geo::Line line(b, geo::Vector(max.x, b.y));
		std::vector<geo::Vector> listOfIntersections;
		for (size_t i = 0; i < APoints.size(); i++)
		{
			geo::Line l(APoints[i], APoints[(i + 1) % APoints.size()]);
			if (geo::Intersecting(l, line))
			{
				if (!listOfIntersections.size() || !std::count(listOfIntersections.begin(), listOfIntersections.end(), (geo::PointOfIntersect(l, line))))
					listOfIntersections.push_back(geo::PointOfIntersect(l, line));
			}
		}
		return listOfIntersections.size() % 2;
	}

	std::vector<geo::Vector> GetIntersectionsBetweenTwoPolygons(
		const PolygonCollider* a, const Transform& ta,
		const PolygonCollider* b, const Transform& tb
	)
	{
		std::vector<geo::Vector> intersections;
		for (size_t i = 0; i < a->points.size(); i++)
		{
			geo::Line ALine(ta.TransformVector(a->points.at(i)),
				ta.TransformVector(a->points.at((i + 1) % a->points.size())));
			for (size_t j = 0; j < b->points.size(); j++)
			{
				geo::Line BLine(tb.TransformVector(b->points.at(j)), 
					tb.TransformVector(b->points.at((j + 1) % b->points.size())));
				if (geo::Intersecting(ALine, BLine))
					intersections.push_back(geo::PointOfIntersect(ALine, BLine));
			}
		}
		return intersections;
	}
}