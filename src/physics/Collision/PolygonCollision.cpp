#include "physics/Collision/Algo.hpp"
#include <iostream>
#define MAX std::numeric_limits<f64>::max()
#define MIN std::numeric_limits<f64>::min()

namespace physics::algo
{
    CollisionPoints SeparatingAxisCheck(
        const geo::Vector2& orthogonal, const std::vector<geo::Vector2>& a,
        const std::vector<geo::Vector2>& b
    );

    std::vector<geo::Vector2> FindCollisionPoints(
        const std::vector<geo::Vector2>& a,
        const std::vector<geo::Vector2>& b
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
        if (a->GetPoints().size() < 3)
            return c;
        std::vector<geo::Vector2> aPoints;
        geo::Vector2 bCenter(ta.rotation.Transpose() * b->center);
        f64 bRadius = b->radius * std::max(tb.scale[0][0], tb.scale[1][1]);
        for (const geo::Vector2& v: a->GetPoints())
            aPoints.push_back(ta.TransformVector(v));
        f64 separation = std::numeric_limits<f64>::min();
        u32 faceNormal = 0;
        // Find edge with minimum penetration
        // Exact concept as using support points in Polygon vs Polygon
        for (u32 i = 0; i < aPoints.size(); i++)
        {
            f64 s = a->GetNormal(i).Dot(bCenter - aPoints[i]);
            if (s > bRadius)
                return c;
            if (s > separation)
            {
                separation = s;
                faceNormal = i;
            }
        }
        geo::Vector2 v1 = aPoints[faceNormal];
        geo::Vector2 v2 = aPoints[(faceNormal + 1) % aPoints.size()];
        if (separation < EPSILON)
        {
            c.normal = (ta.rotation * a->GetNormals()[faceNormal]);
            c.points.push_back(c.normal * bRadius + tb.position);
            c.depth = bRadius;
            c.hasCollision = true;
            return c;
        }
        
        f64 dot1 = (bCenter - v1).Dot(v2 - v1);
        f64 dot2 = (bCenter - v2).Dot(v1 - v2);
        c.depth = bRadius - separation;
        if (dot1 <= 0.0)
        {
            if (geo::DistanceSquared(bCenter, v1) > SQRD(bRadius))
                return c;
            geo::Vector2 n = v1 - bCenter;
            n.Normalize();
            c.normal = -n;
            c.points.push_back(v1);
        }

        else if (dot2 <= 0.0)
        {
            if (geo::DistanceSquared(bCenter, v2) > SQRD(bRadius))
                return c;
            geo::Vector2 n = v2 - bCenter;
            n.Normalize();
            c.normal = -n;
            c.points.push_back(v2);
        }
        
        else
        {
            geo::Vector2 n = a->GetNormal(faceNormal);
            if ((bCenter - v1).Dot(n) > bRadius)
                return c;
            n = ta.rotation * n;
            c.normal = n;
            c.points.push_back(-n * bRadius + tb.position);
        }
        c.hasCollision = true;
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
            const PolygonCollider* b, const Transform& tb)
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
        return c;
    };

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
                    c.normal = tmp.normal;
                }
                for (auto p: tmp.points)
                    c.points.push_back(p);
            }
        }
        if (avg)
            c.depth = avg / (f64)c.points.size();
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

    CollisionPoints SeparatingAxisCheck(
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

    f64 FindMaxSeparation(size_t* edgeIndex, const PolygonCollider* a, const Transform& ta,
        const PolygonCollider* b, const Transform& tb)
    {
        size_t count1 = a->GetPoints().size();
        size_t count2 = b->GetPoints().size();
        Transform t = tb * ta;
        size_t bestIndex = 0;
        f64 maxseparation = std::numeric_limits<f64>::min();
        for (size_t i = 0; i < count1; i++)
        {
            geo::Vector2 n = t.rotation * a->GetNormal(i);
            geo::Vector2 v1 = t.TransformVector(a->GetPoint(i));
            f64 si = std::numeric_limits<f64>::max();
            for (size_t j = 0; j < count2; j++)
            {
                f64 sij = n.Dot(b->GetPoint(j) - v1);
                si = std::min(si, sij);
                if (si > maxseparation)
                {
                    maxseparation = si;
                    bestIndex = i;
                }
            }
        }
        *edgeIndex = bestIndex;
        return maxseparation;
    }

    bool VectorInCircle(const geo::Vector2& a, const CircleCollider* b,
    const Transform& tb)
    {
        return geo::DistanceSquared(a, tb.TransformVector(b->center)) <= SQRD(b->radius * std::max(tb.scale[0][0], tb.scale[1][1]));
    }

    i32 Clip(geo::Vector2 n, f64 c, geo::Vector2* face)
    {
        u32 sp = 0;
        geo::Vector2 out[2] = {
            face[0],
            face[1]
        };

        f64 d1 = n.Dot(face[0]) - c;
        f64 d2 = n.Dot(face[1]) - c;
        if (d1 <= 0.0)
            out[sp++] = face[0];
        if (d2 <= 0.0)
            out[sp++] = face[1];
        if (d1 * d2 <= 0.0)
        {
            f64 alpha = d1 / (d1 - d2);
            out[sp] = face[0] + alpha * (face[1] - face[0]);
            ++sp;
        }
        face[0] = out[0];
        face[1] = out[1];
        assert(sp != 3);
        return sp;
    }

    void FindIncidentFace(geo::Vector2* v, const PolygonCollider* refPoly, const Transform& refTransform, const PolygonCollider* incPoly, const Transform& incTransform, size_t refIndex)
    {
        geo::Vector2 refNormal = refPoly->GetNormals()[refIndex];
        refNormal = refTransform.rotation * refNormal;
        refNormal = incTransform.rotation.Transpose() * refNormal;
        size_t incidentFace = 0;
        f64 minDot = std::numeric_limits<f64>::min();
        for (size_t i = 0; i < incPoly->GetPoints().size(); i++)
        {
            f64 dot = refNormal.Dot(incPoly->GetNormals()[i]);
            if (dot < minDot)
            {
                minDot = dot;
                incidentFace = i;
            }
        }
        v[0] = incTransform.TransformVector(incPoly->GetPoint(incidentFace));
        incidentFace = incidentFace + 1 < incPoly->GetPoints().size() ? incidentFace + 1 : 0;
        v[1] = incTransform.TransformVector(incPoly->GetPoint(incidentFace));
    }

    f64 FindAxisLeastPenetration(size_t* faceIndex, const PolygonCollider* a, const Transform& ta, const PolygonCollider* b, const Transform& tb)
    {
        f64 bestDist = std::numeric_limits<f64>::min();
        size_t bestIndex = 0;
        for (size_t i = 0; i < a->GetPoints().size(); i++)
        {
            geo::Vector2 n = a->GetNormal(i);
            geo::Vector2 nw = ta.rotation * n;
            geo::Matrix2 buT = tb.rotation.Transpose();
            n = buT * nw;
            geo::Vector2 s = b->SupportPoint(-n);
            geo::Vector2 v = a->GetPoint(i);
            v = ta.TransformVector(v);
            v -= tb.position;
            v = buT * v;
            f64 d = n.Dot(s - v);
            if (d > bestDist)
            {
                bestDist = d;
                bestIndex = i;
            }
        }
        *faceIndex = bestIndex;
        return bestDist;
    }
}