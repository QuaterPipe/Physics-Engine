#include "../../include/physics/Collision/Algo.hpp"
#include <iostream>
#define MAX std::numeric_limits<f64>::max()
#define MIN std::numeric_limits<f64>::min()

namespace physics::algo
{
    CollisionPoints SeparatingAxisCheck(
        const geo::Vector2& orthogonal, const std::vector<geo::Vector2>& a,
        const std::vector<geo::Vector2>& b
    );

    bool VectorInPolygon(
        const std::vector<geo::Vector2> points,
        const geo::Vector2& b
    );

    CollisionPoints PolygonCircleCollision(
        const PolygonCollider* a, const Transform& ta,
        const CircleCollider* b, const Transform& tb
    )
    {
        CollisionPoints c;
        c.hasCollision = false;
        if (!a || !b)
            return c;
        if (a->points.size() < 3)
            return c;
        std::vector<geo::Vector2> aPoints;
        geo::Vector2 bCenter(tb.TransformVector(b->center));
        f64 bRadius = b->radius;
        for (const geo::Vector2& v: a->points)
            aPoints.push_back(ta.TransformVector(v));
        
        // closest projection of Circle b's center onto each of polygon a's lines 
        geo::Vector2 closestProjection = geo::Vector2::Infinity;
        // closest vertex in Polygon a to Circle b
        geo::Vector2 closestVertex = geo::Vector2::Infinity;
        for (size_t i = 0; i < aPoints.size(); i++)
        {
            geo::Line aLine(aPoints[(i + 1) % aPoints.size()], aPoints[i]);
            geo::Vector2 proj = geo::Vector2::Projection(bCenter, aLine);
            if (geo::DistanceSquared(proj, bCenter) < geo::DistanceSquared(closestProjection, bCenter) && 
                aLine.VectorIsOnLine(proj))
                closestProjection = proj;
            if (geo::DistanceSquared(aPoints[i], bCenter) < geo::DistanceSquared(closestVertex, bCenter) &&
                VectorInCircle(aPoints[i], b, tb))
                closestVertex = aPoints[i];
        }
        if (closestProjection == geo::Vector2::Infinity && closestVertex == geo::Vector2::Infinity)
            return c;
        bool circInPoly = VectorInPolygon(aPoints, bCenter);
        // if there is no vertex of the polygon in the circle
        if (closestVertex == geo::Vector2::Infinity)
        {
            if (circInPoly)
            {
                c.a = closestProjection;
                geo::Vector2 centerToProjNorm = (c.a - bCenter).Normalized();
                c.b = bCenter + (-centerToProjNorm * bRadius);
                c.normal = (c.b - c.a).Normalized();
                c.depth = geo::Distance(c.a, c.b);
                c.hasCollision = true;
                return c;
            }
            else if (geo::DistanceSquared(closestProjection, bCenter) <= SQRD(bRadius))
            {
                c.a = closestProjection;
                geo::Vector2 centerToProjNorm = (c.a - bCenter).Normalized();
                c.b = bCenter + (centerToProjNorm * bRadius);
                c.normal = (c.b - c.a).Normalized();
                c.depth = geo::Distance(c.a, c.b);
                c.hasCollision = true;
                return c;
            }
        }
        else if (closestVertex != geo::Vector2::Infinity)
        {
            if (circInPoly)
            {
                if (geo::DistanceSquared(closestVertex, bCenter) < geo::DistanceSquared(closestProjection, bCenter))
                    c.a = closestVertex;
                else
                    c.a = closestProjection;
                geo::Vector2 centerToProjNorm = (c.a - bCenter).Normalized();
                c.b = bCenter + (-centerToProjNorm * bRadius);
                c.normal = (c.b - c.a).Normalized();
                c.depth = geo::Distance(c.a, c.b);
                c.hasCollision = true;
                return c;
            }
            else
            {
                if (geo::DistanceSquared(closestVertex, bCenter) < geo::DistanceSquared(closestProjection, bCenter))
                    c.a = closestVertex;
                else
                    c.a = closestProjection;
                geo::Vector2 centerToProjNorm = (c.a - bCenter).Normalized();
                c.b = bCenter + (centerToProjNorm * bRadius);
                c.normal = (c.b - c.a).Normalized();
                c.depth = geo::Distance(c.a, c.b);
                c.hasCollision = true;
                return c;
            }
        }
        return c;
    }

    bool VectorInPolygon(
        const std::vector<geo::Vector2> points,
        const geo::Vector2& b)
    {
        /*
        * All this does is draw a (horizontal)line from b to the farthest point : a, if the amount
        * of intersections with the polygon is even, b is not inside if it is odd b is inside.
        */
        geo::Vector2 max = -geo::Vector2::Infinity;
        for (geo::Vector2 v: points)
        {
            if (max.x < v.x)
                max = v;
        }
        max.x++;
        geo::Line line(b, geo::Vector2(max.x, b.y));
        std::vector<geo::Vector2> listOfIntersections;
        for (size_t i = 0; i < points.size(); i++)
        {
            geo::Line l(points[i], points[(i + 1) % points.size()]);
            if (geo::Intersecting(l, line))
            {
                if (!listOfIntersections.size() || !std::count(listOfIntersections.begin(), listOfIntersections.end(), (geo::PointOfIntersect(l, line))))
                    listOfIntersections.push_back(geo::PointOfIntersect(l, line));
            }
        }
        return listOfIntersections.size() % 2;
    }

    std::pair<geo::Vector2, geo::Vector2> FindCollisionPoints(
        const std::vector<geo::Vector2>& a,
        const std::vector<geo::Vector2>& b,
        const geo::Vector2& orthogonal
    )
    {
        geo::Vector2 ACentroid = geo::Centroid(a);
        geo::Vector2 BCentroid = geo::Centroid(b);
        geo::Vector2 closestVec = geo::Vector2::Infinity;
        for (geo::Vector2 p: a)
        {
            if (geo::DistanceSquared(closestVec, BCentroid) >
                geo::DistanceSquared(p, BCentroid) &&
                VectorInPolygon(b, p))
            {
                closestVec = p;
            }
        }
        if (closestVec != geo::Vector2::Infinity)
        {
            geo::Vector2 finalProj(geo::Vector2::Infinity);
            for (size_t i = 0; i < b.size(); i++)
            {
                geo::Line bLine(b[i], b[(i + 1) % b.size()]);
                geo::Vector2 proj = geo::Vector2::Projection(closestVec, bLine);
                geo::Vector2 diff = closestVec - proj;
                if (fabs(diff.GetMagnitudeSquared() - orthogonal.GetMagnitudeSquared()) <
                    fabs((closestVec - finalProj).GetMagnitudeSquared() - orthogonal.GetMagnitudeSquared()))
                {
                    if (!diff.Cross(orthogonal) && (diff.Dot(orthogonal) < 0))
                        finalProj = proj;
                }
            }
            if (finalProj != geo::Vector2::Infinity)
                return std::pair<geo::Vector2, geo::Vector2>(closestVec, finalProj);
        }
        closestVec = geo::Vector2::Infinity;
        for (geo::Vector2 p: b)
        {
            if (geo::DistanceSquared(closestVec, ACentroid) >
                geo::DistanceSquared(p, ACentroid) &&
                VectorInPolygon(a, p))
            {
                closestVec = p;
            }
        }
        if (closestVec != geo::Vector2::Infinity)
        {
            geo::Vector2 finalProj(geo::Vector2::Infinity);
            for (size_t i = 0; i < a.size(); i++)
            {
                geo::Line aLine(a[i], a[(i + 1) % a.size()]);
                geo::Vector2 proj = geo::Vector2::Projection(closestVec, aLine);
                geo::Vector2 diff = closestVec - proj;
                if (fabs(diff.GetMagnitudeSquared() - orthogonal.GetMagnitudeSquared()) <
                    fabs((closestVec - finalProj).GetMagnitudeSquared() - orthogonal.GetMagnitudeSquared()))
                {
                    if (!diff.Cross(orthogonal) && (diff.Dot(orthogonal) < 0))
                        finalProj = proj;
                }
            }
            return std::pair<geo::Vector2, geo::Vector2>(closestVec, finalProj);
        }
        std::pair<geo::Vector2, geo::Vector2> pr;
        return pr;
    }

    CollisionPoints PolygonPolygonCollision(
            const PolygonCollider* a, const Transform& ta,
            const PolygonCollider* b, const Transform& tb)
    {
        CollisionPoints c;
        c.hasCollision = false;
        if (!a || !b)
            return c;
        if (a->points.size() < 3 || b->points.size() < 3)
            return c;
        std::vector<geo::Vector2> aPoints, bPoints;
        for (size_t i = 0; i < a->points.size(); i++)
            aPoints.push_back(ta.TransformVector(a->points[i]));
        for (size_t i = 0; i < b->points.size(); i++)
            bPoints.push_back(tb.TransformVector(b->points[i]));

        std::vector<geo::Vector2> pushVectors;
        for (size_t i = 0; i < a->points.size() + b->points.size(); i++)
        {
            geo::Vector2 edge;
            if (i < a->points.size())
                edge = aPoints[(i + 1) % a->points.size()] - aPoints[i];
            else
                edge = bPoints[(i + 1) % b->points.size()] - bPoints[i];
            geo::Vector2 ortho(-edge.y, edge.x);
            CollisionPoints clsn = SeparatingAxisCheck(ortho, aPoints, bPoints);
            if (clsn.hasCollision == true)
                return c;
            else
                pushVectors.push_back(clsn.normal);
        }
        if (!pushVectors.size())
            return c;
        // collision with the smallest push vector.
        geo::Vector2 mpv = geo::Vector2::Infinity;
        for (const geo::Vector2& pv: pushVectors)
        {
            if (mpv.Dot(mpv) > pv.Dot(pv))
                mpv = pv;
        }
        geo::Vector2 d = tb.TransformVector(b->GetCenter()) - ta.TransformVector(a->GetCenter());
        if (d.Dot(mpv) > 0)
            mpv = -mpv;
        c.hasCollision = true;
        c.normal = mpv.Normalized();
        c.depth = mpv.GetMagnitude();
        auto inter = FindCollisionPoints(aPoints, bPoints, mpv);
        c.a = inter.first;
        c.b = inter.second;
        return c;
    };

    CollisionPoints SeparatingAxisCheck(
        const geo::Vector2& orthogonal, const std::vector<geo::Vector2>& a,
        const std::vector<geo::Vector2>& b
    )
    {
        CollisionPoints c;
        c.hasCollision = false;
        f64 minA = MAX, maxA = MIN;
        f64 minB = MAX, maxB = MIN;
        for (const geo::Vector2& v: a)
        {
            f64 projection = v.Dot(orthogonal);
            minA = std::min(minA, projection);
            maxA = std::max(maxA, projection);
        }
        for (const geo::Vector2& v: b)
        {
            f64 projection = v.Dot(orthogonal);
            minB = std::min(minB, projection);
            maxB = std::max(maxB, projection);
        }

        if (maxA >= minB && maxB >= minA)
        {
            f64 d = std::min(maxB - minA, maxA - minB);
            f64 e = d / orthogonal.Dot(orthogonal) + EPSILON;
            geo::Vector2 pv = e * orthogonal;
            c.normal = pv;
            return c;
        }
        c.hasCollision = true;
        return c;
    }

    CollisionPoints PolygonBoxCollision(
        const PolygonCollider* a, const Transform& ta,
        const BoxCollider* b, const Transform& tb)
    {
        CollisionPoints c;
        if (!a || !b)
            return c;
        PolygonCollider bb = PolygonCollider(*b);
        return PolygonPolygonCollision(a, ta, &bb, tb);
    }

    CollisionPoints PolygonMeshCollision(
        const PolygonCollider* a, const Transform& ta,
        const MeshCollider* b, const Transform& tb
    )
    {
        CollisionPoints c;
        if (!a || !b)
            return c;
        for (const Collider* ptr: b->colliders)
        {
            c = ptr->TestCollision(tb, a, ta);
            if (c.hasCollision)
                return c;
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
        if (a->points.size() < 3)
            return false;
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

}