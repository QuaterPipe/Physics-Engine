#include "physics/Collision/Algo.hpp"
#include <iostream>
#define MAX std::numeric_limits<f64>::max()
#define MIN std::numeric_limits<f64>::min()

namespace physics::algo
{
    struct SATCheck
    {
        bool failed;
        geo::Vector2 normal;
    };

    SATCheck SeparatingAxis(
        const geo::Vector2& orthogonal, const geo::Vector2* a,
        const geo::Vector2* b, size_t aSize, size_t bSize
    );

    std::vector<geo::Vector2> FindCollisionPoints(
        const geo::Vector2* a,
        const geo::Vector2* b, size_t aSize, size_t bSize, geo::Vector2& norm
    );

    bool VectorInPolygon(
        const geo::Vector2* points,
        const geo::Vector2& b, size_t pointsSize
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
        if (a->GetPointCount() < 3)
            return c;
        size_t aSize = a->GetPointCount();
        geo::Vector2* aPoints = new geo::Vector2[aSize];
        for (size_t i = 0; i < aSize; i++)
            aPoints[i] = ta.TransformVector(a->GetPoint(i));
        geo::Vector2 bCenter = tb.TransformVector(b->center);
        f64 bRadius = b->radius * geo::Max(tb.GetScale().x, tb.GetScale().y);
        bool centerInA = VectorInPolygon(aPoints, bCenter, aSize), polyInB = true;
        geo::Vector2* projections = new geo::Vector2[aSize];
        size_t projInd = 0;
        for (size_t i = 0; i < aSize; i++)
            polyInB &= b->Contains(aPoints[i], tb);
        for (int i = 0; i < aSize; i++)
        {
            geo::Line l(aPoints[(i + 1) % aSize], aPoints[i]);
            geo::Vector2 proj = geo::Vector2::Projection(bCenter, l);
            if (l.VectorIsOnLine(proj))
                projections[projInd++] = proj;
        }
        geo::Vector2 closest = geo::Vector2::Infinity;
        f64 minDis = std::numeric_limits<f64>::infinity();
        for (size_t i = 0; i < aSize; i++)
        {
            if (geo::DistanceSquared(bCenter, aPoints[i]) < minDis)
            {
                minDis = geo::DistanceSquared(bCenter, aPoints[i]);
                closest = aPoints[i];
            }
        }
        for (size_t i = 0; i < projInd; i++)
        {
            if (geo::DistanceSquared(bCenter, projections[i]) < minDis && projections[i] != bCenter)
            {
                minDis = geo::DistanceSquared(bCenter, projections[i]);
                closest = projections[i];
            }
        }
        if (!b->Contains(closest, tb) && !centerInA)
        {
            delete[] aPoints;
            delete[] projections;
            return c;
        }
        if (centerInA || polyInB || closest != geo::Vector2::Infinity)
        {
            c.hasCollision = true;
            c.points.resize(2);
            c.points[0] = closest;
            c.depth = (geo::Distance(bCenter, closest)) + bRadius;
            if (centerInA)
            {
                c.normal = -(closest - bCenter).Normalized();
                c.points[1] = c.normal * bRadius + bCenter;
            }
            else
            {
                c.normal = (closest - bCenter).Normalized();
                c.points[1] = c.normal * bRadius + bCenter;
                c.depth = bRadius - geo::Distance(bCenter, closest);
            }
        }
        if (!flipped)
            c.normal = -c.normal;
        delete[] aPoints;
        delete[] projections;
        return c;
    }

    bool VectorInPolygon(
        const geo::Vector2* points,
        const geo::Vector2& b, size_t pointsSize)
    {
        f64 x = b.x, y = b.y;
        bool inside = false;
        geo::Vector2 p1, p2;
        for (int i = 1; i < pointsSize; i++)
        {
            p1 = points[i - 1];
            p2 = points[i];
            if (y > geo::Min(p1.y, p2.y) && y <= geo::Max(p1.y, p2.y))
            {
                if (x <= geo::Max(p1.x, p2.x))
                {
                    f64 x_inter = (y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
                    if (p1.x == p2.x || x <= x_inter)
                    {
                        inside = !inside;
                    }
                }
            }
        }
        p1 = points[pointsSize - 1];
        p2 = points[0];
        if (y > geo::Min(p1.y, p2.y) && y <= geo::Max(p1.y, p2.y))
        {
            if (x <= geo::Max(p1.x, p2.x))
            {
                f64 x_inter = (y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
                if (p1.x == p2.x || x <= x_inter)
                {
                    inside = !inside;
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
        if (a->GetPointCount() < 3 || b->GetPointCount() < 3)
            return c;
        size_t aSize = a->GetPointCount(), bSize = b->GetPointCount();
        geo::Vector2* aPoints = new geo::Vector2[aSize];
        geo::Vector2* bPoints = new geo::Vector2[bSize];
        for (size_t i = 0; i < aSize; i++)
            aPoints[i] = ta.TransformVector(a->GetPoint(i));
        for (size_t i = 0; i < bSize; i++)
            bPoints[i] = tb.TransformVector(b->GetPoint(i));
        std::vector<geo::Vector2> pushVectors;
        pushVectors.reserve(aSize + bSize);
        for (size_t i = 0; i < aSize + bSize; i++)
        {
            geo::Vector2 edge;
            if (i < aSize)
                edge = aPoints[(i + 1) % aSize] - aPoints[i];
            else
                edge = bPoints[(i - aSize + 1) % bSize] - bPoints[i - aSize];
            geo::Vector2 ortho(-edge.y, edge.x);
            SATCheck check = SeparatingAxis(ortho, aPoints, bPoints, aSize, bSize);
            if (check.failed == true)
            {
                delete[] aPoints;
                delete[] bPoints;
                return c;
            }
            else
                pushVectors.push_back(check.normal);
        }
        if (!pushVectors.size())
        {
            delete[] aPoints;
            delete[] bPoints;
            return c;
        }
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
        c.normal = -mpv.Normalized();
        if (!c.normal.GetMagnitudeSquared())
            c.normal.Set(1, 0);
        if (flipped)
            c.normal = -c.normal;
        c.depth = mpv.GetMagnitude();
        std::vector<geo::Vector2> inter = FindCollisionPoints(aPoints, bPoints, aSize, bSize, c.normal);
        c.points = inter; 
        delete[] aPoints;
        delete[] bPoints;
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

    geo::Vector2 SupportPoint(const geo::Vector2* points, size_t size, geo::Vector2 direction)
    {
        f64 bestProj = std::numeric_limits<f64>::min();
        geo::Vector2 bestVertex(geo::Vector2::Infinity);
        for (size_t i = 0; i < size; i++)
        {
            f64 proj = points[i].Dot(direction);
            if (proj > bestProj)
            {
                bestVertex = points[i];
                bestProj = proj;
            }
        }
        return bestVertex;
    }

    std::vector<geo::Vector2> FindCollisionPoints(
        const geo::Vector2* a,
        const geo::Vector2* b, size_t aSize, size_t bSize, geo::Vector2& norm
    )
    {
        std::vector<geo::Vector2> points;
        points.reserve(geo::Min(aSize, bSize));
        for (size_t i = 0; i < aSize; i++)
        {
            if (VectorInPolygon(b, a[i], bSize))
                points.push_back(a[i]);
        }
        for (size_t i = 0; i < bSize; i++)
        {
            if (VectorInPolygon(a, b[i], aSize))
                points.push_back(b[i]);
        }
        return points;
    }

    SATCheck SeparatingAxis(
        const geo::Vector2& orthogonal, const geo::Vector2* a,
        const geo::Vector2* b, size_t aSize, size_t bSize
    )
    {
        SATCheck s;
        s.failed = false;
        f64 minA = MAX, maxA = MIN;
        f64 minB = MAX, maxB = MIN;
        for (size_t i = 0; i < aSize; i++)
        {
            f64 projection = a[i].Dot(orthogonal);
            minA = geo::Min(minA, projection);
            maxA = geo::Max(maxA, projection);
        }
        for (size_t i = 0; i < bSize; i++)
        {
            f64 projection = b[i].Dot(orthogonal);
            minB = geo::Min(minB, projection);
            maxB = geo::Max(maxB, projection);
        }
        if (maxA >= minB && maxB >= minA)
        {
            f64 d = geo::Min(maxB - minA, maxA - minB);
            f64 e = d / orthogonal.Dot(orthogonal) + EPSILON;
            geo::Vector2 pv = e * orthogonal;
            s.normal = pv;
            return s;
        }
        s.failed = true;
        return s;
    }
}