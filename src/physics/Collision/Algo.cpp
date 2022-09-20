#include "../../include/physics/Collision/Algo.hpp"
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
		const geo::Vector2 BCenter = tb.TransformVector(b->center);
		const geo::Vector2 APos = ta.TransformVector(a->position);
		if (geo::DistanceSquared(BCenter, APos) <= SQRD(b->radius))
		{
			c.contactPoints.push_back(APos);
			geo::Vector2 tmp = (APos - BCenter).Normalized();
			c.contactPoints.push_back(tmp * b->radius + BCenter);
			c.depth = geo::Distance(c.contactPoints[0], c.contactPoints[1]);
			c.normal = (c.contactPoints[1] - c.contactPoints[0]).Normalized();
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
		geo::Vector2 APos = ta.TransformVector(a->position);
		if (!VectorInPolygon(b, tb, APos))
			return c;
		std::vector<geo::Vector2> BPoints;
		for (const geo::Vector2& v: b->points)
		{
			BPoints.push_back(tb.TransformVector(v + b->pos));
		}
		c.contactPoints.push_back(geo::Vector2::Infinity);
		c.contactPoints.push_back(APos);
		geo::Vector2 closest = geo::Vector2::Infinity;
		for (size_t i = 0; i < BPoints.size(); i++)
		{
			geo::Line l(BPoints[i], BPoints[(i + 1) % BPoints.size()]);
			geo::Vector2 p = geo::Vector2::Projection(APos, l);
			if (geo::DistanceSquared(closest, APos) > geo::DistanceSquared(p, APos))
			{
				if (l.VectorIsOnLine(p))
					closest = p;
			}
		}
		c.contactPoints[0] = closest;
		c.depth = geo::Distance(c.contactPoints[0], c.contactPoints[1]);
		c.normal = (c.contactPoints[1] - c.contactPoints[0]).Normalized();
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
		PolygonCollider bb = PolygonCollider(b->pos, b->pos,
			geo::Vector2(b->x + b->width, b->y), 
			geo::Vector2(b->x + b->width, b->y + b->height), {geo::Vector2(b->x , b->y + b->height)});
		return PointPolygonCollision(a, ta, &bb, tb);
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
			c.contactPoints.push_back(ta.TransformVector(a->position));
			c.contactPoints.push_back(tb.TransformVector(b->position));
			c.depth = geo::Distance(c.contactPoints[0], c.contactPoints[1]);
			c.normal = (c.contactPoints[1] - c.contactPoints[0]).Normalized();
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
		const geo::Vector2 ACenter = ta.TransformVector(a->center);
		const geo::Vector2 BCenter = tb.TransformVector(b->center);
		f64 r = geo::DistanceSquared(ACenter, BCenter);
		// If the sum of their radii is greater than or equal to the distance between their centers
		if (SQRD(a->radius + b->radius) >= r)
		{
			geo::Line l(ACenter, BCenter);
			c.contactPoints.push_back(l.GetVectorAlongLine(b->radius, false));
			c.contactPoints.push_back(l.GetVectorAlongLine(a->radius));
			c.depth = geo::Distance(c.contactPoints[0], c.contactPoints[1]);
			c.normal = c.contactPoints[1] - c.contactPoints[0];
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
		PolygonCollider bb = PolygonCollider(b->pos, b->pos,
			geo::Vector2(b->x + b->width, b->y), 
			geo::Vector2(b->x + b->width, b->y + b->height), {geo::Vector2(b->x , b->y + b->height)});
		return PolygonCircleCollision(&bb, tb, a, ta);
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
		const geo::Vector2 BCenter = tb.TransformVector(b->center);
		size_t pointInCircleCount = 0;
		size_t lineInCircleCount = 0;
		for (size_t i = 0; i < a->points.size(); i++)
		{
			const geo::Line side(ta.TransformVector(a->points[i]), ta.TransformVector(a->points[(i + 1) % a->points.size()]));
			if (LinePassesThroughCircle(side, b, tb) && side.VectorIsOnLine(geo::Vector2::Projection(BCenter, side)))
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
			return PolygonLinePenetratingCircle(a, ta, b, tb);
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
		std::vector<geo::Vector2> edges(a->points.size() + b->points.size());
		std::vector<geo::Vector2> orthogonals(a->points.size() + b->points.size());
		std::vector<geo::Vector2> APoints;
		std::vector<geo::Vector2> BPoints;
		for (size_t i = 0; i < a->points.size(); i++)
		{
			APoints.push_back(ta.TransformVector(a->points[i]));
			geo::Vector2 v{
				ta.TransformVector(a->points[(i + 1) % a->points.size()]) -
				ta.TransformVector(a->points[i])
			};
			edges.push_back(v);
			orthogonals.emplace_back(-v.y, v.x);
		}
		for (size_t i = 0; i < b->points.size(); i++)
		{
			BPoints.push_back(tb.TransformVector(b->points[i]));
			geo::Vector2 v{
				tb.TransformVector(b->points[(i + 1) % b->points.size()]) -
				tb.TransformVector(b->points[i])
			};
			edges.push_back(v);
			orthogonals.emplace_back(-v.y, v.x);
		}
		std::vector<geo::Vector2> pushVectors;
		for (size_t i = 0; i < orthogonals.size(); i++)
		{
 			CollisionPoints tmp = SeparatingAxisCheck(a, ta, b, tb, orthogonals[i]);
			if (tmp.hasCollision)
				return c;
			else
				pushVectors.push_back(tmp.normal);
		}
		geo::Vector2 minPush;
		f64 minDis = MAX;
		for (geo::Vector2& v: pushVectors)
		{
			if (minDis > v.GetMagnitudeSquared() && (v.GetMagnitude() > EPSILON))
			{
				minPush = v;
				minDis = v.GetMagnitudeSquared();
			}
		}
		auto mean = [&](std::vector<geo::Vector2> vecs){
			geo::Vector2 total;
			for (geo::Vector2 v: vecs)
				total += v;
			total /= vecs.size();
			return total;
		};
		geo::Vector2 d = mean(BPoints) - mean(APoints);
		if (d.Dot(minPush) > 0)
			minPush = -minPush;
		c.contactPoints = PointOfIntersect(a, ta, b, tb);
		c.normal = minPush.Normalized();
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
		PolygonCollider bb = PolygonCollider(b->pos, b->pos,
			geo::Vector2(b->x + b->width, b->y), 
			geo::Vector2(b->x + b->width, b->y + b->height), {geo::Vector2(b->x , b->y + b->height)});
		return PolygonPolygonCollision(a, ta, &bb, tb);
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
		PolygonCollider bb = PolygonCollider(b->pos, b->pos,
			geo::Vector2(b->x + b->width, b->y), 
			geo::Vector2(b->x + b->width, b->y + b->height), {geo::Vector2(b->x , b->y + b->height)});
		PolygonCollider aa = PolygonCollider(a->pos, a->pos,
			geo::Vector2(a->x + a->width, a->y), 
			geo::Vector2(a->x + a->width, a->y + a->height), {geo::Vector2(a->x , a->y + a->height)});
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
		const geo::Vector2 BCenter = tb.TransformVector(b->center);
		if (geo::DistanceSquared(a.a, BCenter) <= SQRD(b->radius))
			return true;
		else if (geo::DistanceSquared(a.b, BCenter) <= SQRD(b->radius))
			return true;
		geo::Vector2 proj = geo::Vector2::Projection(BCenter, a);
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
		const geo::Vector2 BCenter = tb.TransformVector(b->center);
		geo::Vector2 closestPoint;
		f64 minDis = MAX;
		for (size_t i = 0; i < a->points.size(); i++)
		{
			const geo::Vector2 vA = ta.TransformVector(a->points[i]);
			const geo::Vector2 vB = ta.TransformVector(a->points[(i + 1) % a->points.size()]);
			const geo::Line side(vA, vB);
			const geo::Vector2 proj = geo::Vector2::Projection(BCenter, side);
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
		c.contactPoints.push_back(closestPoint);
		c.contactPoints.push_back(geo::Line(BCenter, closestPoint).GetVectorAlongLine(b->radius));
		c.depth = geo::Distance(closestPoint, c.contactPoints[0]);
		c.normal = (c.contactPoints[0] - c.contactPoints[1]).Normalized();
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
		const geo::Vector2 BCenter = tb.TransformVector(b->center);
		std::vector<geo::Vector2> APoints;
		
		geo::Vector2 farthestPoint;
		f64 maxDis = MIN;
		// transforming the vertexes, and finding the farthest vertex from the circle's center
		for (geo::Vector2 v: a->points)
		{
			geo::Vector2 tmp = ta.TransformVector(v);
			if (!VectorInCircle(tmp, b, tb))
				return c;
			if (geo::DistanceSquared(tmp, BCenter) > maxDis)
			{
				c.contactPoints.push_back(tmp);
				farthestPoint = tmp;
				maxDis = geo::DistanceSquared(tmp, BCenter);
			}
			APoints.push_back(tmp);
		}

		// projecting the circle's center onto each line, and finding the farthest projection
		for (size_t i = 0; i < APoints.size(); i++)
		{
			geo::Line side(APoints[i], APoints[(i + 1) % APoints.size()]);
			geo::Vector2 Proj = geo::Vector2::Projection(BCenter, side);
			if (side.VectorIsOnLine(Proj) && geo::DistanceSquared(BCenter, Proj) > maxDis)
			{
				farthestPoint = Proj;
				maxDis = geo::DistanceSquared(BCenter, Proj);
			}
		}
		geo::Vector2 proj  = geo::Line(BCenter, farthestPoint).GetVectorAlongLine(b->radius);
		c.contactPoints.push_back(proj);
		c.depth = geo::Distance(farthestPoint, proj);
		c.hasCollision = true;
		c.normal = (farthestPoint - proj).Normalized();
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
		const geo::Vector2 BCenter = tb.TransformVector(b->center);
		geo::Vector2 closestPoint;
		f64 minDis = MAX;
		for (size_t i = 0; i < a->points.size(); i++)
		{
			geo::Vector2 v = ta.TransformVector(a->points[i]);
			if (geo::DistanceSquared(v, BCenter) < minDis)
			{
				closestPoint = v;
				minDis = geo::DistanceSquared(v, BCenter);
			}
		}
		if (minDis > b->radius)
			return c;
		c.contactPoints.push_back(geo::Line(BCenter, closestPoint).GetVectorAlongLine(minDis + b->radius));
		c.contactPoints.push_back(closestPoint);
		c.normal = (closestPoint - c.contactPoints[0]).Normalized();
		c.depth = minDis + b->radius;
		c.hasCollision = true;
		return c;
	}

	CollisionPoints PolygonLinePenetratingCircle(
		const PolygonCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	)
	{
		CollisionPoints c;
		if (!a || !b)
			return c;
		const geo::Vector2 BCenter = tb.TransformVector(b->center);
		geo::Vector2 closestPoint;
		f64 minDis = MAX;
		for (size_t i = 0; i < a->points.size(); i++)
		{
			const geo::Vector2 vA = ta.TransformVector(a->points[i]);
			const geo::Vector2 vB = ta.TransformVector(a->points[(i + 1) % a->points.size()]);
			const geo::Line side(vA, vB);
			const geo::Vector2 Proj = geo::Vector2::Projection(BCenter, side);
			if (geo::Distance(BCenter, Proj) < minDis && LinePassesThroughCircle(side, b, tb) &&
				side.VectorIsOnLine(Proj))
			{
				closestPoint = geo::Vector2::Projection(BCenter, side);
				c.contactPoints.push_back(closestPoint);
				minDis = geo::DistanceSquared(closestPoint, BCenter);
			}
			if (geo::DistanceSquared(vA, BCenter) < minDis)
			{
				closestPoint = vA;
				c.contactPoints.push_back(closestPoint);
				minDis = geo::DistanceSquared(vA, BCenter);
			}
		}
		//// there actually IS a collision here, but this function does not handle this type of collision
		//if (AllLinesInCircle)
		//	return c;
		geo::Vector2 proj = geo::Line(BCenter, closestPoint).GetVectorAlongLine(b->radius);
		c.contactPoints.push_back(proj);
		c.depth = geo::Distance(closestPoint, proj);
		c.normal = (closestPoint - proj).Normalized();
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
		const geo::Vector2 BCenter = tb.TransformVector(b->center);
		geo::Vector2 Point;
		for (geo::Vector2 v: a->points)
		{
			Point = ta.TransformVector(v);
			if (VectorInCircle(Point, b, tb))
			{
				c.contactPoints.push_back(Point);
				c.contactPoints.push_back(geo::Line(BCenter, Point).GetVectorAlongLine(b->radius));
				c.depth = geo::Distance(Point, *(c.contactPoints.end() - 1));
				c.normal = (*(c.contactPoints.end() - 1) - Point).Normalized();
				c.hasCollision = true;
			}	
		}
		return c;
	}

	bool VectorInPolygon(
		const PolygonCollider* a, const Transform& ta,
		const geo::Vector2& b)
	{
		/*
		* All this does is draw a (horizontal)line from b to the farthest point : a, if the amount
		* of intersections with the polygon is even, b is not inside if it is odd b is inside.
		*/
		if (a->points.size() <3)/* ily too <3*/ {return false;}
		std::vector<geo::Vector2> APoints;
		for (const geo::Vector2& v: a->points)
			APoints.push_back(ta.TransformVector(v + a->pos));
		geo::Vector2 max = -geo::Vector2::Infinity;
		for (geo::Vector2 v: APoints)
		{
			if (max.x < v.x)
				max = v;
		}
		max.x++;
		geo::Line line(b, geo::Vector2(max.x, b.y));
		std::vector<geo::Vector2> listOfIntersections;
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

	bool VectorInCircle(const geo::Vector2& a, const CircleCollider* b,
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
			geo::Vector2 ACenter = ta.TransformVector(a->center);
			geo::Vector2 proj = geo::Vector2::Projection(ACenter, b);
			c.contactPoints.push_back(proj);
			c.contactPoints.push_back(ACenter);
			c.normal = c.contactPoints[1] - c.contactPoints[0];
			c.depth = geo::Distance(proj, ACenter);
		}
		return c;
	}

	// thanks to: https://hackmd.io/@US4ofdv7Sq2GRdxti381_A/ryFmIZrsl
	CollisionPoints SeparatingAxisCheck(
		const PolygonCollider* a, const Transform& ta,
		const PolygonCollider* b, const Transform& tb,
		const geo::Vector2& orthogonal
	)
	{
		CollisionPoints c;
		if (!a || !b)
			return c;
		f64 amin = MAX, amax = MIN, bmin = MAX, bmax = MIN;
		for (geo::Vector2 v: a->points)
		{
			f64 proj = ta.TransformVector(v).Dot(orthogonal);
			amin = proj < amin ? proj : amin;
			amax = proj > amax ? proj : amax;
		}
		for (geo::Vector2 v: b->points)
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

	std::vector<geo::Vector2> PointOfIntersect(
		const PolygonCollider* a, const Transform &ta,
		const PolygonCollider* b, const Transform& tb
	)
	{
		if (!a || !b)
			return std::vector<geo::Vector2>();
		if (a->points.size() + b->points.size() < 6)
			return std::vector<geo::Vector2>();
		std::vector<geo::Vector2> contacts;
		for (auto p: a->points)
		{
			if (VectorInPolygon(b, tb, ta.TransformVector(p)))
				contacts.push_back(ta.TransformVector(p));
		}
		for (auto p: b->points)
		{
			if (VectorInPolygon(a, ta, tb.TransformVector(p)))
				contacts.push_back(tb.TransformVector(p));
		}
		return contacts;
	}
	CollisionPoints PolygonCircleContacts(
		const PolygonCollider* a, const Transform& ta,
		const CircleCollider* b, const Transform& tb
	)
	{

	}
}