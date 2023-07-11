#include "geometry/Math.hpp"

namespace geo
{

	
	Line ClosestLine(std::vector<geo::Line> lines, Vector2 vector) noexcept
	{
		Line closestLine = Line();
		f64 minDis = std::numeric_limits<f64>::infinity();
		for (Line l: lines)
		{
			if (minDis > DistanceSquared(l, vector))
			{
				closestLine = l;
				minDis = DistanceSquared(l, vector);
			}
		}
		return closestLine;
	}

	f64 Degrees(f64 radians) noexcept
	{
		return radians * (180 / M_PI);
	}

	f64 Distance(const Vector2 &a, const Vector2 &b) noexcept
	{
		f64 dis = SQRD(b.x - a.x) + SQRD(b.y - a.y);
		return sqrt(dis);
	}

	f64 Distance(const Line& a, const Vector2& b) noexcept
	{
		const Vector2 proj = Vector2::Projection(b, a);
		return Distance(proj, b);
	}

	f64 Distance(const Vector3& a, const Vector3& b) noexcept
	{
		f64 dis = (SQRD(b.x - a.x) + SQRD(b.y - a.y) + SQRD(b.z - a.z));
		return sqrt(dis);
	}

	f64 DistanceSquared(const Vector2& a, const Vector2& b) noexcept
	{
		f64 dis = (SQRD(b.x - a.x) + SQRD(b.y - a.y));
		return dis;
	}

	f64 DistanceSquared(const Line& a, const Vector2& b) noexcept
	{
		const Vector2 proj = Vector2::Projection(b, a);
		return DistanceSquared(proj, b);
	}

	f64 DistanceSquared(const Vector3& a, const Vector3& b) noexcept
	{
		f64 dis = (SQRD(b.x - a.x) + SQRD(b.y - a.y) + SQRD(b.z - a.z));
		if (dis < 0) {dis *= -1;}
		return dis;
	}

	bool Equal(f64 a, f64 b) noexcept
	{
		return std::abs(a - b) <= EPSILON;
	}

	f64 FastSqrt(f64 x) noexcept
	{
		f32 n = (f32)x;
	   	static union{int i; float f;} u;
	   	u.i = 0x5F375A86 - (*(int*)&n >> 1);
	   	return (f64)((int(3) - n * u.f * u.f) * n * u.f * 0.5f);
	}

	f64 GetAngle(const Vector2 &a, const Vector2 &b, const Vector2 &c) noexcept
	{
		f64 result = atan2(c.y - b.y, c.x - b.x) - atan2(a.y - b.y, a.x - b.x);
		result = result < 0 ? -result : (M_PI * 2) - result;
		return result;
	}

	f64 GetAngle(f64 slope) noexcept
	{
		return atan(slope);
	}	

	f64 GetAngle(const Vector2& center, const Vector2& Vector) noexcept
	{
		return atan2(Vector.y - center.y, Vector.x - center.x);
	}

	Vector2 GetVectorOnCircle(const Vector2& center, f64 radius, f64 angle) noexcept
	{
		f64 x = center.x + (fabs(radius) * cos(angle));
		f64 y = center.y + (fabs(radius) < 0 ? -radius : radius * sin(angle));
		return Vector2(x, y);
	}

	f64 GetSlope(const Vector2& a, const Vector2& b) noexcept
	{
		if (a.y == b.y || a.x == b.x)
			return 0;
		return (b.y - a.y) / (b.x - a.x);
	}

	bool Intersecting(const Line& a, const Line& b, bool isInfLine) noexcept
	{
		return PointOfIntersect(a, b, isInfLine) != Vector2::Infinity;
	}

	f64 Lerp(f64 a, f64 b, f64 t)
	{
		return a + (b - a) * t;
	}

	Vector2 PointOfIntersect(const Line& la, const Line& lb, bool isInfLine) noexcept
	{
		const geo::Vector2& A = la.a, & B = la.b, & C = lb.a, & D = lb.b;
		const f64 top = (D.x - C.x) * (A.y - C.y) - (D.y - C.y) * (A.x - C.x);
		const f64 bottom = (D.y - C.y) * (B.x - A.x) - (D.x - C.x) * (B.y - A.y);
		const f64 t = top / bottom;
		const Vector2 v(Lerp(A.x, B.x, t), Lerp(A.y, B.y, t));
		if (!isInfLine && (!la.VectorIsOnLine(v) || !lb.VectorIsOnLine(v)))
			return Vector2::Infinity;
		return v;
	}

	f64 Radians(f64 degrees) noexcept
	{
		return degrees * (M_PI / 180);
	}

	Vector2 Centroid(const Vector2* start, const Vector2* end) noexcept
	{
		
		Vector2 first = *start;
		Vector2 last = *end;
		std::vector<Vector2> points;
		points.insert(points.end(), start, end);
		if (points.size())
		{
			if (first.x != last.x || first.y != last.y)
			{
				points.push_back(first);
			}
			f64 twiceArea = 0, x = 0, y = 0, f = 0;
			Vector2 p1, p2;
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
			return Vector2(x / f + first.x, y / f + first.y);
		}
		else
			return Vector2::Origin;
	}

	Vector2 Centroid(const std::vector<geo::Vector2>& vertexes) noexcept
	{
		
		Vector2 first = *vertexes.begin();
		Vector2 last = *(vertexes.end() - 1);
		std::vector<Vector2> points;
		points.insert(points.end(), vertexes.begin(), vertexes.end());
		if (points.size())
		{
			if (first.x != last.x || first.y != last.y)
			{
				points.push_back(first);
			}
			f64 twiceArea = 0, x = 0, y = 0, f = 0;
			Vector2 p1, p2;
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
			return Vector2(x / f + first.x, y / f + first.y);
		}
		else
			return Vector2::Origin;
	}
}
