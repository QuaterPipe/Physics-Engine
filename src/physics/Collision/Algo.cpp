#include "../../include/physics/Collision/Algo.hpp"
#include "../../include/physics/Tools/OstreamOverloads.hpp"
#include <iostream>
#define MAX std::numeric_limits<f64>::max()
#define MIN std::numeric_limits<f64>::min()

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
		if (!VectorInPolygon(b, tb, APos))
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
		if (geo::DistanceSquared(ta.TransformVector(a->position), tb.TransformVector(b->position)) < SQRD(EPSILON))
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
			c.b= l.GetVectorAlongLine(a->radius);
			c.a = l.GetVectorAlongLine(b->radius, false);
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
		const geo::Vector BCenter = tb.TransformVector(b->center);
		size_t pointInCircleCount = 0;
		size_t lineInCircleCount = 0;
		for (size_t i = 0; i < a->points.size(); i++)
		{
			const geo::Line side(ta.TransformVector(a->points[i]), ta.TransformVector(a->points[(i + 1) % a->points.size()]));
			if (LinePassesThroughCircle(side, b, tb) && side.VectorIsOnLine(geo::Vector::Projection(BCenter, side)))
			{
				lineInCircleCount++;
			}
			if (VectorInCircle(ta.TransformVector(a->points[i]), b, tb))
			{
				pointInCircleCount++;
			}
		}
		if (pointInCircleCount == a->points.size())
			return PolygonInsideCircle(a, ta, b, tb);
		if (lineInCircleCount)
			return PolygonLineInCircle(a, ta, b, tb);
		if (pointInCircleCount)
			return PolygonVertexInCircle(a, ta, b, tb);
		c = CircleCenterInPolygon(a, ta, b, tb);
		if (c.hasCollision)
			return c;
		else
			return CircleInsidePolygon(a, ta, b, tb);
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
		std::vector<geo::Vector> edges(a->points.size() + b->points.size());
		std::vector<geo::Vector> orthogonals(a->points.size() + b->points.size());
		std::vector<geo::Vector> APoints;
		std::vector<geo::Vector> BPoints;
		for (size_t i = 0; i < a->points.size(); i++)
		{
			APoints.push_back(ta.TransformVector(a->points[i]));
			geo::Vector v{
				ta.TransformVector(a->points[(i + 1) % a->points.size()]) -
				ta.TransformVector(a->points[i])
			};
			edges.push_back(v);
			orthogonals.emplace_back(-v.y, v.x);
		}
		for (size_t i = 0; i < b->points.size(); i++)
		{
			BPoints.push_back(tb.TransformVector(b->points[i]));
			geo::Vector v{
				tb.TransformVector(b->points[(i + 1) % b->points.size()]) -
				tb.TransformVector(b->points[i])
			};
			edges.push_back(v);
			orthogonals.emplace_back(-v.y, v.x);
		}
		std::vector<geo::Vector> pushVectors;
		for (size_t i = 0; i < orthogonals.size(); i++)
		{
 			CollisionPoints tmp = SeparatingAxisCheck(a, ta, b, tb, orthogonals[i]);
			if (tmp.hasCollision)
				return c;
			else
				pushVectors.push_back(tmp.normal);
		}
		geo::Vector minPush;
		f64 minDis = MAX;
		for (geo::Vector& v: pushVectors)
		{
			if (minDis > v.GetMagnitudeSquared() && (v.GetMagnitude() > EPSILON))
			{
				minPush = v;
				minDis = v.GetMagnitudeSquared();
			}
		}
		auto mean = [&](std::vector<geo::Vector> vecs){
			geo::Vector total;
			for (geo::Vector v: vecs)
				total += v;
			total /= vecs.size();
			return total;
		};
		geo::Vector d = mean(BPoints) - mean(APoints);
		if (d.Dot(minPush) > 0)
			minPush = -minPush;
		c.normal = minPush.Normalized();
		c.a = PointOfIntersect(a, ta, b, tb);
		c.b = c.a + c.normal;
		c.depth = minPush.GetMagnitude();
		c.hasCollision = minPush.GetMagnitude();
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
		const geo::Vector BCenter = tb.TransformVector(b->center);
		if (geo::DistanceSquared(a.a, BCenter) <= SQRD(b->radius))
			return true;
		else if (geo::DistanceSquared(a.b, BCenter) <= SQRD(b->radius))
			return true;
		geo::Vector proj = geo::Vector::Projection(BCenter, a);
		return (geo::DistanceSquared(proj, BCenter) <= SQRD(b->radius));
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
		const geo::Vector BCenter = tb.TransformVector(b->center);
		geo::Vector closestPoint;
		f64 minDis = MAX;
		for (size_t i = 0; i < a->points.size(); i++)
		{
			const geo::Vector vA = ta.TransformVector(a->points[i]);
			const geo::Vector vB = ta.TransformVector(a->points[(i + 1) % a->points.size()]);
			const geo::Line side(vA, vB);
			const geo::Vector proj = geo::Vector::Projection(BCenter, side);
			if (geo::DistanceSquared(proj, BCenter) < minDis && side.VectorIsOnLine(proj))
			{
				closestPoint = proj;
				minDis = geo::DistanceSquared(proj, BCenter);
			}
			if (geo::DistanceSquared(vA, BCenter) < minDis)
			{
				closestPoint = vA;
				minDis = geo::DistanceSquared(vA, BCenter);
			}
		}
		if (minDis == MAX || !VectorInPolygon(a, ta, BCenter))
			return c;
		c.a = closestPoint;
		c.b = geo::Line(BCenter, c.a).GetVectorAlongLine(b->radius);
		c.depth = geo::Distance(c.a, c.b);
		c.normal = (c.a - c.b).Normalized();
		c.hasCollision = true;
		return c;
	}

	CollisionPoints PolygonInsideCircle(
		const PolygonCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		if (!a || !b) return c;
		const geo::Vector BCenter = tb.TransformVector(b->center);
		std::vector<geo::Vector> APoints;
		
		geo::Vector farthestPoint;
		f64 maxDis = MIN;
		// transforming the vertexes, and finding the farthest vertex from the circle's center
		for (geo::Vector v: a->points)
		{
			geo::Vector tmp = ta.TransformVector(v);
			if (!VectorInCircle(tmp, b, tb))
				return c;
			if (geo::DistanceSquared(tmp, BCenter) > maxDis)
			{
				farthestPoint = tmp;
				maxDis = geo::DistanceSquared(tmp, BCenter);
			}
			APoints.push_back(tmp);
		}

		// projecting the circle's center onto each line, and finding the farthest projection
		for (size_t i = 0; i < APoints.size(); i++)
		{
			geo::Line side(APoints[i], APoints[(i + 1) % APoints.size()]);
			geo::Vector Proj = geo::Vector::Projection(BCenter, side);
			if (side.VectorIsOnLine(Proj) && geo::DistanceSquared(BCenter, Proj) > maxDis)
			{
				farthestPoint = Proj;
				maxDis = geo::DistanceSquared(BCenter, Proj);
			}
		}
		c.a = farthestPoint;
		c.b = geo::Line(BCenter, c.a).GetVectorAlongLine(b->radius);
		c.depth = geo::Distance(c.a, c.b);
		c.hasCollision = true;
		c.normal = (c.a - c.b).Normalized();
		return c;
	}

	CollisionPoints CircleCenterInPolygon(
		const PolygonCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		if (!a || !b)
			return c;
		const geo::Vector BCenter = tb.TransformVector(b->center);
		geo::Vector closestPoint;
		f64 minDis = MAX;
		for (size_t i = 0; i < a->points.size(); i++)
		{
			geo::Vector v = ta.TransformVector(a->points[i]);
			if (geo::DistanceSquared(v, BCenter) < minDis)
			{
				closestPoint = v;
				minDis = geo::DistanceSquared(v, BCenter);
			}
		}
		if (minDis > b->radius)
			return c;
		c.b = closestPoint;
		c.a = geo::Line(BCenter, c.b).GetVectorAlongLine(minDis + b->radius);
		c.normal = (c.b - c.a).Normalized();
		c.depth = minDis + b->radius;
		c.hasCollision = true;
		return c;
	}

	CollisionPoints PolygonLineInCircle(
		const PolygonCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		if (!a || !b)
			return c;
		const geo::Vector BCenter = tb.TransformVector(b->center);
		geo::Vector closestPoint;
		f64 minDis = MAX;
		for (size_t i = 0; i < a->points.size(); i++)
		{
			const geo::Vector vA = ta.TransformVector(a->points[i]);
			const geo::Vector vB = ta.TransformVector(a->points[(i + 1) % a->points.size()]);
			const geo::Line side(vA, vB);
			const geo::Vector Proj = geo::Vector::Projection(BCenter, side);
			if (geo::Distance(BCenter, Proj) < minDis && LinePassesThroughCircle(side, b, tb) &&
				side.VectorIsOnLine(Proj))
			{
				closestPoint = geo::Vector::Projection(BCenter, side);
				minDis = geo::DistanceSquared(closestPoint, BCenter);
			}
			if (geo::DistanceSquared(vA, BCenter) < minDis)
			{
				closestPoint = vA;
				minDis = geo::DistanceSquared(vA, BCenter);
			}
		}
		// there actually IS a collision here, but this function does not handle this type of collision
		//if (AllLinesInCircle)
		//	return c;
		c.a = closestPoint;
		c.b = geo::Line(BCenter, c.a).GetVectorAlongLine(b->radius);
		c.depth = geo::Distance(c.a, c.b);
		c.normal = (c.a - c.b).Normalized();
		c.hasCollision = true;
		return c;
	}

	CollisionPoints PolygonVertexInCircle(
		const PolygonCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		if (!a || !b)
			return c;
		const geo::Vector BCenter = tb.TransformVector(b->center);
		geo::Vector Point;
		for (geo::Vector v: a->points)
		{
			Point = ta.TransformVector(v);
			if (VectorInCircle(Point, b, tb))
			{
				c.a = Point;
				c.b = geo::Line(BCenter, c.a).GetVectorAlongLine(b->radius);
				c.depth = geo::Distance(c.a, c.b);
				c.normal = (c.b - c.a).Normalized();
				c.hasCollision = true;
				return c;
			}	
		}
		return c;
	}

	bool VectorInPolygon(
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

	bool VectorInCircle(const geo::Vector& a, const CircleCollider* b,
	const Transform& tb)
	{
		return geo::DistanceSquared(a, tb.TransformVector(b->center)) <= SQRD(b->radius);
	}

	CollisionPoints LineInCircle(
		const CircleCollider* a, const Transform& ta,
		const geo::Line& b
	)
	{
		CollisionPoints c;
		c.hasCollision = VectorInCircle(b.a, a, ta) && VectorInCircle(b.b, a, ta);
		if (c.hasCollision)
		{
			geo::Vector ACenter = ta.TransformVector(a->center);
			geo::Vector proj = geo::Vector::Projection(ACenter, b);
			c.a = proj;
			c.b = ACenter;
			c.normal = c.b - c.a;
			c.depth = geo::Distance(c.a, c.b);
		}
		return c;
	}

	// thanks to: https://hackmd.io/@US4ofdv7Sq2GRdxti381_A/ryFmIZrsl
	CollisionPoints SeparatingAxisCheck(
		const PolygonCollider* a, const Transform& ta,
		const PolygonCollider* b, const Transform& tb,
		const geo::Vector& orthogonal
	)
	{
		CollisionPoints c;
		if (!a || !b)
			return c;
		f64 amin = MAX, amax = MIN, bmin = MAX, bmax = MIN;
		for (geo::Vector v: a->points)
		{
			f64 proj = ta.TransformVector(v).Dot(orthogonal);
			amin = proj < amin ? proj : amin;
			amax = proj > amax ? proj : amax;
		}
		for (geo::Vector v: b->points)
		{
			f64 proj = tb.TransformVector(v).Dot(orthogonal);
			bmin = proj < bmin ? proj : bmin;
			bmax = proj > bmax ? proj : bmax;
		}
		if (amax >= bmin && bmax >= amin)
		{
			f64 d = std::min(bmax - amin, amax - bmin);
			if (((bmin <= amin) && (amin <= amax) && (amax <= bmax)) ||
				((amin <= bmin) && (bmin <= bmax) && (bmax <= amax)))
				d = std::min(fabs(amin - bmin), fabs(amax - bmax));
			f64 dOverOSquared = d / orthogonal.Dot(orthogonal) + 1e-10;
			c.hasCollision = false;
			c.normal = dOverOSquared * orthogonal;
		}
		else
			c.hasCollision = true;
		return c;
	}

	geo::Vector PointOfIntersect(
		const PolygonCollider* a, const Transform &ta,
		const PolygonCollider* b, const Transform& tb
	)
	{
		if (!a || !b)
			return geo::Vector::Infinity;
		if (a->points.size() + b->points.size() < 6)
			return geo::Vector::Infinity;
		geo::Vector farthestVec;
		f64 farthestDis = MIN;
		const geo::Vector centroidA = geo::Centroid(&*a->points.begin(), &*a->points.end());
		for (size_t i = 0; i < a->points.size(); i++)
		{
			const geo::Vector p = ta.TransformVector(a->points[i]);
			if (VectorInPolygon(b, tb, p))
			{
				if (geo::DistanceSquared(p, centroidA) > farthestDis)
				{
					farthestVec = p;
					farthestDis = geo::DistanceSquared(p, centroidA);
				}
			}
		}
		const geo::Vector centroidB = geo::Centroid(&*b->points.begin(), &*b->points.end());
		for (size_t i = 0; i < b->points.size(); i++)
		{
			const geo::Vector p = tb.TransformVector(b->points[i]);
			if (VectorInPolygon(a, ta, p))
			{
				if (geo::DistanceSquared(p, centroidB) > farthestDis)
				{
					farthestVec = p;
					farthestDis = geo::DistanceSquared(p, centroidB);
				}
			}
		}
		if (farthestDis == MIN)
			return geo::Vector::Infinity;
		return farthestVec;
	}
}