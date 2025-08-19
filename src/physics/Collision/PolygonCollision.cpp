#include "physics/Collision/Algo.hpp"
#include <iostream>
#define MAX std::numeric_limits<f64>::max()
#define MIN -MAX

namespace physics::algo
{    
    Manifold PolygonCircleCollision(
        const PolygonCollider* a, const Transform& ta,
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
        bool centerInA = VectorInPolygon(aPoints, bCenter, aSize), polyInB = true;
        Vector2* projections = new Vector2[aSize];
        size_t projInd = 0;
        for (size_t i = 0; i < aSize; i++)
            polyInB &= b->Contains(aPoints[i], tb);
        for (int i = 0; i < aSize; i++)
        {
            Line l(aPoints[(i + 1) % aSize], aPoints[i]);
            Vector2 proj = Vector2::Projection(bCenter, l);
            if (l.VectorIsOnLine(proj))
                projections[projInd++] = proj;
        }
        Vector2 closest = Vector2::Infinity;
        f64 minDis = std::numeric_limits<f64>::infinity();
        for (size_t i = 0; i < aSize; i++)
        {
            if (DistanceSquared(bCenter, aPoints[i]) < minDis)
            {
                minDis = DistanceSquared(bCenter, aPoints[i]);
                closest = aPoints[i];
            }
        }
        for (size_t i = 0; i < projInd; i++)
        {
            if (DistanceSquared(bCenter, projections[i]) < minDis && projections[i] != bCenter)
            {
                minDis = DistanceSquared(bCenter, projections[i]);
                closest = projections[i];
            }
        }
        if (!b->Contains(closest, tb) && !centerInA)
        {
            delete[] projections;
            delete[] aPoints;
            return c;
        }
        if (centerInA || polyInB || closest != Vector2::Infinity)
        {
            c.hasCollision = true;
            c.points.resize(2, Vector2());
            c.points[0] = closest;
            c.depth = (Distance(bCenter, closest)) + bRadius;
            if (centerInA)
            {
                c.normal = -(closest - bCenter).Normalized();
                c.points[1] = c.normal * bRadius + bCenter;
            }
            else
            {
                c.normal = (closest - bCenter).Normalized();
                c.points[1] = c.normal * bRadius + bCenter;
                c.depth = bRadius - Distance(bCenter, closest);
            }
        }
        if (!flipped)
            c.normal = -c.normal;
        c.pointCount = 2;
        delete[] projections;
        delete[] aPoints;
        return c;
    }

    bool VectorInPolygon(
        const Vector2* points,
        const Vector2& b, size_t pointsSize)
    {
        f64 x = b.x, y = b.y;
        bool inside = false;
        Vector2 p1, p2;
        for (int i = 1; i < pointsSize; i++)
        {
            p1 = points[i - 1];
            p2 = points[i];
            if (y > Min(p1.y, p2.y) && y <= Max(p1.y, p2.y))
            {
                if (x <= Max(p1.x, p2.x))
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
        if (y > Min(p1.y, p2.y) && y <= Max(p1.y, p2.y))
        {
            if (x <= Max(p1.x, p2.x))
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

    Vector3 SAT(Vector2* aPoints, size_t aSize, Vector2* bPoints, size_t bSize) noexcept
    {
        f64 minOverlap = MAX;
        size_t minInd = 0;
        Vector2 smallestAxis;
        Vector2* edges = new Vector2[aSize + bSize];
        for (size_t i = 0; i < aSize; i++)
        {
            edges[i] = aPoints[(i + 1) % aSize] - aPoints[i];
            edges[i].Set(-edges[i].y, edges[i].x);
            edges[i].Normalize();
        }
        for (size_t i = 0; i < bSize; i++)
        {
            edges[i + aSize] = bPoints[(i + 1) % bSize] - bPoints[i];
            edges[i + aSize].Set(-edges[i + aSize].y, edges[i + aSize].x);
            edges[i + aSize].Normalize();
        }
        for (size_t i = 0; i < aSize + bSize; i++)
        {
            if (edges[i].GetMagnitudeSquared() <= SQRD(EPSILON))
                continue;
            f64 minA = MAX, maxA = MIN;
            for (size_t j = 0; j < aSize; j++)
            {
                f64 p = aPoints[j].Dot(edges[i]);
                minA = Min(minA, p);
                maxA = Max(maxA, p);
            }

            f64 minB = MAX, maxB = MIN;
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

    Manifold PolygonPolygonCollision(
        const PolygonCollider* a, const Transform& ta,
        const PolygonCollider* b, const Transform& tb, bool flipped
    )
    {
        Manifold c;
        c.hasCollision = false;
        if (!a || !b)
            return c;
        if (a->GetPointCount() < 3 || b->GetPointCount() < 3)
            return c;
        size_t aSize = a->GetPointCount(), bSize = b->GetPointCount();
        Vector2* aPoints = new Vector2[aSize];
        Vector2* bPoints = new Vector2[bSize];
        for (size_t i = 0; i < aSize; i++)
            aPoints[i] = ta.TransformVector(a->GetPoint(i));
        for (size_t i = 0; i < bSize; i++)
            bPoints[i] = tb.TransformVector(b->GetPoint(i));
        
        Vector3 check = SAT(aPoints, aSize, bPoints, bSize);
        if (check == Vector3::Infinity)
        {
            delete[] aPoints;
            delete[] bPoints;
            return c;
        }
        Vector2 axis(check.x, check.y);

        for (size_t i = 0; i < aSize; i++)
            if (VectorInPolygon(bPoints, aPoints[i], bSize))
                c.points.push_back(aPoints[i]);
        for (size_t i = 0; i < bSize; i++)
            if (VectorInPolygon(aPoints, bPoints[i], aSize))
                c.points.push_back(bPoints[i]);
        delete[] aPoints;
        delete[] bPoints;
        c.pointCount = c.points.size();
        c.depth = axis.GetMagnitudeExact();
        c.normal = axis / c.depth;
        c.hasCollision = true;
        if (flipped)
            c.normal = -c.normal;
        return c;
    };

    Manifold PolygonBoxCollision(
        const PolygonCollider* a, const Transform& ta,
        const BoxCollider* b, const Transform& tb, bool flipped
    )
    {
        Manifold c;
        if (!a || !b)
            return c;
        PolygonCollider bb = PolygonCollider(*b);
        return PolygonPolygonCollision(a, ta, &bb, tb, flipped);
    }

    Manifold PolygonMeshCollision(
        const PolygonCollider* a, const Transform& ta,
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