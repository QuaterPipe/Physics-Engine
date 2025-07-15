#include "physics/Collision/Algo.hpp"
#include <iostream>
#define MAX std::numeric_limits<f64>::max()
#define MIN -MAX

namespace physics::algo
{

    bool BiasGreaterThan(f64 a, f64 b)
    {
        const f64 k_biasRelative = 0.95;
        const f64 k_biasAbsolute = 0.01;
        return a >= b * k_biasRelative + a * k_biasAbsolute;
    }

    int Clip(const geo::Vector2& n, f64 c, geo::Vector2* face)
    {
        int sp = 0;
        geo::Vector2 out[2] = {
            face[0],
            face[1]
        };
        f64 d1 = n.Dot(face[0]) - c;
        f64 d2 = n.Dot(face[1]) - c;
        if (d1 <= 0.0) out[sp++] = face[0];
        if (d2 <= 0.0) out[sp++] = face[1];
        if (d1 * d2 < 0.0)
        {
            f64 alpha = d1 / (d1 - d2);
            out[sp] = face[0] + alpha * (face[1] - face[0]);
            sp++;
        }
        face[0] = out[0];
        face[1] = out[1];
        assert(sp != 3);
        return sp;
    }

    f64 FindAxisLeastPenetration(size_t* faceIndex, const PolygonCollider* a, const Transform& ta, const PolygonCollider* b, const Transform& tb)
    {
        f64 bestDistance = MIN;
        size_t bestIndex = 0;
        geo::Matrix2 aRot = ta.GetRotation();
        geo::Matrix2 bRot = tb.GetRotation();
        geo::Matrix2 bRotxScale = bRot * geo::Matrix2(tb.GetScale().x, 0, 0, tb.GetScale().y);
        geo::Matrix2 binvRotxScale = bRotxScale.GetTranspose();
        geo::Matrix3 bTransformMatrix = tb.GetTransformationMatrix();
        for (size_t i = 0; i < a->GetPointCount(); i++)
        {
            geo::Vector2 n = a->GetNormal(i);
            geo::Vector2 nw = aRot * n;
            geo::Matrix2 buT = bRot.GetTranspose();
            n = buT * nw;
            geo::Vector2 s = b->SupportPoint(-n);
            geo::Vector2 v = a->GetPoint(i);
            v = ta.TransformVector(v);
            v.x -= bTransformMatrix(0, 2);
            v.y -= bTransformMatrix(1, 2);
            v = binvRotxScale * v;
            f64 d = n.Dot(s - v);
            if (d > bestDistance)
            {
                bestDistance = d;
                bestIndex = i;
            }
        }
        *faceIndex = bestIndex;
        return bestDistance;
    }

    void FindIncidentFace(geo::Vector2* v, const PolygonCollider* refPoly, const Transform& refTransform, const PolygonCollider* incPoly, const Transform& incTransform, size_t referenceIndex)
    {
        geo::Vector2 referenceNormal = refPoly->GetNormal(referenceIndex);
        referenceNormal = refTransform.GetRotation() * referenceNormal;
        referenceNormal = incTransform.GetRotation().GetTranspose() * referenceNormal;
        size_t incidentFace = 0;
        f64 minDot = MAX;
        for (size_t i = 0; i < incPoly->GetPointCount(); i++)
        {
            f64 dot = referenceNormal.Dot(incPoly->GetNormal(i));
            if (dot < minDot)
            {
                minDot = dot;
                incidentFace = i;
            }
        }
        v[0] = incTransform.TransformVector(incPoly->GetPoint(incidentFace));
        incidentFace = incidentFace + 1 >= (size_t)incPoly->GetPointCount() ? 0 : incidentFace + 1;
        v[1] = incTransform.TransformVector(incPoly->GetPoint(incidentFace));

    }

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
        geo::Vector2 aPoints[MAX_POLYGONCOLLIDER_SIZE];
        for (size_t i = 0; i < aSize; i++)
            aPoints[i] = ta.TransformVector(a->GetPoint(i));
        geo::Vector2 bCenter = tb.TransformVector(b->center);
        f64 bRadius = b->radius * geo::Max(tb.GetScale().x, tb.GetScale().y);
        bool centerInA = VectorInPolygon(aPoints, bCenter, aSize), polyInB = true;
        geo::Vector2 projections[MAX_POLYGONCOLLIDER_SIZE];
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
            return c;
        }
        if (centerInA || polyInB || closest != geo::Vector2::Infinity)
        {
            c.hasCollision = true;
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
        c.pointCount = 2;
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
        geo::Vector2 aPoints [MAX_POLYGONCOLLIDER_SIZE];
        geo::Vector2 bPoints[MAX_POLYGONCOLLIDER_SIZE];
        for (size_t i = 0; i < aSize; i++)
            aPoints[i] = ta.TransformVector(a->GetPoint(i));
        for (size_t i = 0; i < bSize; i++)
            bPoints[i] = tb.TransformVector(b->GetPoint(i));
        size_t faceA;
        f64 penetrationA = FindAxisLeastPenetration(&faceA, a, ta, b, tb);
        if (penetrationA >= 0.0)
            return c;
        size_t faceB;
        f64 penetrationB = FindAxisLeastPenetration(&faceB, b, tb, a, ta);
        if (penetrationB >= 0.0)
            return c;
        size_t referenceIndex;
        bool flip;
        const PolygonCollider* refPoly;
        const PolygonCollider* incPoly;
        Transform refTransform;
        Transform incTransform;
        if (BiasGreaterThan(penetrationA, penetrationB))
        {
            refPoly = a;
            refTransform = ta;
            incPoly = b;
            incTransform = tb;
            referenceIndex = faceA;
            flip = false;
        }

        else
        {
            refPoly = b;
            refTransform = tb;
            incPoly = a;
            incTransform = ta;
            referenceIndex = faceB;
            flip = true;
        }

        geo::Vector2 incidentFace[2];
        FindIncidentFace(incidentFace, refPoly, refTransform, incPoly, incTransform, referenceIndex);
        geo::Vector2 v1 = refPoly->GetPoint(referenceIndex);
        referenceIndex = referenceIndex + 1 == refPoly->GetPointCount() ? 0 : referenceIndex + 1;
        geo::Vector2 v2 = refPoly->GetPoint(referenceIndex);
        v1 = refTransform.TransformVector(v1);
        v2 = refTransform.TransformVector(v2);
        geo::Vector2 sidePlaneNormal = (v2 - v1);
        sidePlaneNormal.Normalize();
        geo::Vector2 refFaceNormal(sidePlaneNormal.y, -sidePlaneNormal.x);
        f64 refC = refFaceNormal.Dot(v1);
        f64 negSide = -sidePlaneNormal.Dot(v1);
        f64 posSide = sidePlaneNormal.Dot(v2);
        if (Clip(-sidePlaneNormal, negSide, (geo::Vector2*)incidentFace) < 2)
            return c;
        if (Clip(sidePlaneNormal, posSide, (geo::Vector2*)incidentFace) < 2)
            return c;
        c.normal = flip ? -refFaceNormal : refFaceNormal;
        size_t cp = 0;
        f64 separation = refFaceNormal.Dot(incidentFace[0]) - refC;
        if (separation <= 0.0)
        {
            c.points[cp] = incidentFace[0];
            c.depth = -separation;
            ++cp;
        }
        else
            c.depth = 0;

        separation = refFaceNormal.Dot(incidentFace[1]) - refC;
        if (separation <= 0.0)
        {
            c.points[cp] = incidentFace[1];
            c.depth += -separation;
            ++cp;
            c.depth /= (f64)cp;
        }

        c.pointCount = cp;
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
        f64 avg = 0;
        for (const Collider* ptr : b->colliders)
        {
            Manifold tmp = ptr->TestCollision(tb, a, ta);
            if (tmp.hasCollision)
            {
                avg += tmp.depth;
                c.hasCollision = true;
                if (c.depth < tmp.depth)
                {
                    c.depth = tmp.depth;
                    c.normal = -tmp.normal;
                }
                for (size_t i = 0; i < tmp.pointCount; i++)
                {
                    if (c.pointCount < MAX_MANIFOLD_POINT_COUNT)
                        c.points[c.pointCount++] = tmp.points[i];
                }
            }
        }
        if (avg)
            c.depth = avg / (f64)c.pointCount;
        if (flipped)
            c.normal = -c.normal;
        return c;
    }
}