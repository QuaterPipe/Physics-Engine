#include "physics/Collision/Algo.hpp"
#include <iostream>
#define MAX std::numeric_limits<f64>::max()
#define MIN std::numeric_limits<f64>::min()

namespace physics::algo
{
    CollisionPoints SeparatingAxis(
        const geo::Vector2& orthogonal, const std::vector<geo::Vector2>& a,
        const std::vector<geo::Vector2>& b
    );

    CollisionPoints SeparatingAxis(
        const geo::Vector2& orthogonal, const std::vector<geo::Vector2>& a,
        const geo::Vector2& bCenter, f64 bRadius
    );

    std::vector<geo::Vector2> FindCollisionPoints(
        const std::vector<geo::Vector2>& a,
        const std::vector<geo::Vector2>& b
    );

    CollisionPoints PolygonCircleCollision(
        const PolygonCollider* a, const Transform& ta,
        const CircleCollider* b, const Transform& tb, bool flipped
    )
    {
        CollisionPoints c;
        c.hasCollision = false;
        if (!a || !b)
            return c;
        if (a->GetPoints().size() < 3)
            return c;
        std::vector<geo::Vector2> aPoints;
        geo::Vector2 centroid;
        for (geo::Vector2 v : a->GetPoints())
        {
            aPoints.push_back(ta.TransformVector(v));
            centroid += *(aPoints.end() - 1);
        }
        centroid /= aPoints.size();
        geo::Vector2 bCenter = b->center + tb.position; // no other members of tb will affect b->center.
        f64 bRadius = b->radius * std::max(tb.scale[0][0], tb.scale[1][1]);
        bool centerInA = a->Contains(bCenter, ta), polyInB = true;
        std::vector<geo::Vector2> projections;
        for (geo::Vector2 v : aPoints)
        {
            polyInB &= b->Contains(v, tb);
        }
        for (int i = 0; i < aPoints.size(); i++)
        {
            geo::Line l(aPoints[(i + 1) % aPoints.size()], aPoints[i]);
            geo::Vector2 proj = geo::Vector2::Projection(bCenter, l);
            if (l.VectorIsOnLine(proj))
                projections.push_back(proj);
        }
        geo::Vector2 closest = geo::Vector2::Infinity;
        f64 minDis = std::numeric_limits<f64>::infinity();
        for (geo::Vector2 v : aPoints)
        {
            if (geo::DistanceSquared(bCenter, v) < minDis)
            {
                minDis = geo::DistanceSquared(bCenter, v);
                closest = v;
            }
        }
        for (geo::Vector2 v : projections)
        {
            if (geo::DistanceSquared(bCenter, v) < minDis)
            {
                minDis = geo::DistanceSquared(bCenter, v);
                closest = v;
            }
        }
        if (!b->Contains(closest, tb) && !centerInA)
            return c;
        if (centerInA || polyInB || closest != geo::Vector2::Infinity)
        {
            c.hasCollision = true;
            c.points.push_back(closest);
            c.depth = (geo::Distance(bCenter, closest)) + bRadius;
            if (centerInA)
            {
                c.normal = -(closest - bCenter).Normalized();
                c.points.push_back(c.normal * bRadius + bCenter);
            }
            else
            {
                c.normal = (closest - bCenter).Normalized();
                c.points.push_back((c.normal) * bRadius + bCenter);
                c.depth = bRadius - geo::Distance(bCenter, closest);
            }
        }
        if (flipped)
            c.normal = -c.normal;
        return c;
    }

    bool VectorInPolygon(
        const std::vector<geo::Vector2> points,
        const geo::Vector2& b)
    {
        f64 x = b.x, y = b.y;
        bool inside = false;
        geo::Vector2 p1, p2;
        for (int i = 1; i <= points.size(); i++)
        {
            p1 = points[i % points.size()];
            p2 = points[(i + 1) % points.size()];
            if (y > std::min(p1.y, p2.y) && y <= std::max(p1.y, p2.y))
            {
                if (x <= std::max(p1.x, p2.x))
                {
                    f64 x_inter = (y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
                    if (p1.x == p2.x || x <= x_inter)
                    {
                        inside = !inside;
                    }
                }
            }
        }
        return inside;
    }

    CollisionPoints PolygonPolygonCollision(
            const PolygonCollider* a, const Transform& ta,
            const PolygonCollider* b, const Transform& tb, bool flipped
    )
    {
        CollisionPoints c;
        c.hasCollision = false;
        if (!a || !b)
            return c;
        if (a->GetPoints().size() < 3 || b->GetPoints().size() < 3)
            return c;
        std::vector<geo::Vector2> aPoints, bPoints;
        for (size_t i = 0; i < a->GetPoints().size(); i++)
            aPoints.push_back(ta.TransformVector(a->GetPoint(i)));
        for (size_t i = 0; i < b->GetPoints().size(); i++)
            bPoints.push_back(tb.TransformVector(b->GetPoint(i)));
        std::vector<geo::Vector2> pushVectors;
        for (size_t i = 0; i < aPoints.size() + bPoints.size(); i++)
        {
            geo::Vector2 edge;
            if (i < aPoints.size())
                edge = aPoints[(i + 1) % aPoints.size()] - aPoints[i];
            else
                edge = bPoints[(i - aPoints.size() + 1) % bPoints.size()] - bPoints[i - aPoints.size()];
            geo::Vector2 ortho(-edge.y, edge.x);
            CollisionPoints clsn = SeparatingAxis(ortho, aPoints, bPoints);
            if (clsn.hasCollision == true)
                return c;
            else
                pushVectors.push_back(clsn.normal);
        }
        if (!pushVectors.size())
            return c;
        // collision with the smallest push vector.
        geo::Vector2 mpv = geo::Vector2::Infinity;
        for (const geo::Vector2& pv : pushVectors)
        {
            if (mpv.Dot(mpv) > pv.Dot(pv))
                mpv = pv;
        }
        geo::Vector2 d = tb.TransformVector(b->GetCenter()) - ta.TransformVector(a->GetCenter());
        if (d.Dot(mpv) > 0)
            mpv = -mpv;
        c.hasCollision = true;
        c.normal = mpv.Normalized();
        if (!c.normal.GetMagnitudeSquared())
            c.normal.Set(1, 0);
        c.depth = mpv.GetMagnitude();
        std::vector<geo::Vector2> inter = FindCollisionPoints(aPoints, bPoints);
        c.points = inter; 
        if (flipped)
            c.normal = -c.normal;
        return c;
    };

    CollisionPoints PolygonBoxCollision(
        const PolygonCollider* a, const Transform& ta,
        const BoxCollider* b, const Transform& tb, bool flipped
    )
    {
        CollisionPoints c;
        if (!a || !b)
            return c;
        PolygonCollider bb = PolygonCollider(*b);
        return PolygonPolygonCollision(a, ta, &bb, tb, flipped);
    }

    CollisionPoints PolygonMeshCollision(
        const PolygonCollider* a, const Transform& ta,
        const MeshCollider* b, const Transform& tb, bool flipped
    )
    {
        CollisionPoints c;
        if (!a || !b)
            return c;
        f64 avg = 0;
        for (const Collider* ptr: b->colliders)
        {
            CollisionPoints tmp = ptr->TestCollision(tb, a, ta);
            if (tmp.hasCollision)
            {
                avg += tmp.depth;
                c.hasCollision = true;
                if (c.depth < tmp.depth)
                {
                    c.depth = tmp.depth;
                    c.normal = -tmp.normal;
                }
                for (auto p: tmp.points)
                    c.points.push_back(p);
            }
        }
        if (avg)
            c.depth = avg / (f64)c.points.size();
        if (flipped)
            c.normal = -c.normal;
        return c;
    }


    std::vector<geo::Vector2> FindCollisionPoints(
        const std::vector<geo::Vector2>& a,
        const std::vector<geo::Vector2>& b
    )
    {
        std::vector<geo::Vector2> points;
        for (geo::Vector2 p : a)
        {
            if (VectorInPolygon(b, p))
                points.push_back(p);
        }
        for (geo::Vector2 p : b)
        {
            if (VectorInPolygon(a, p))
            points.push_back(p);
        }
        return points;
    }

    CollisionPoints SeparatingAxis(
        const geo::Vector2& orthogonal, const std::vector<geo::Vector2>& a,
        const std::vector<geo::Vector2>& b
    )
    {
        CollisionPoints c;
        c.hasCollision = false;
        f64 minA = MAX, maxA = MIN;
        f64 minB = MAX, maxB = MIN;
        for (const geo::Vector2& v : a)
        {
            f64 projection = v.Dot(orthogonal);
            minA = std::min(minA, projection);
            maxA = std::max(maxA, projection);
        }
        for (const geo::Vector2& v : b)
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

    CollisionPoints SeparatingAxis(
        const geo::Vector2& orthogonal, const std::vector<geo::Vector2>& a,
        const geo::Vector2& bCenter, f64 bRadius
    )
    {
        CollisionPoints c;
        c.hasCollision = false;
        f64 minA = MAX, maxA = MIN;
        f64 minB = MAX, maxB = MIN;
        for (const geo::Vector2& v : a)
        {
            f64 projection = v.Dot(orthogonal);
            minA = std::min(minA, projection);
            maxA = std::max(maxA, projection);
        }
        {
            f64 projection = bCenter.Dot(orthogonal);
            minB = projection - bRadius;
            maxB = projection + bRadius;
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
}