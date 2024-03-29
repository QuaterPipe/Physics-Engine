#include "geometry/Math.hpp"
#include <iostream>

namespace geo
{
	i64 Abs(i64 x) noexcept
	{
		if (x < 0)
			return -x;
		else
			return x;
	}

	f64 Abs(f64 x) noexcept
	{
		if (x < 0)
			return -x;
		else
			return x;
	}

	Vector2 Centroid(const std::vector<geo::Vector2>& vertices) noexcept
	{
		Vector2 first = *vertices.begin();
		Vector2 last = *(vertices.end() - 1);
		Vector2* points = new Vector2[vertices.size() + 1];
		for (size_t i = 0; i < vertices.size(); i++)
			points[i] = vertices[i];
		int ind = vertices.size();
		int add = 0;
		if (vertices.size())
		{
			if (first.x != last.x || first.y != last.y)
			{
				points[ind] = first;
				add++;
			}
			f64 twiceArea = 0, x = 0, y = 0, f = 0;
			Vector2 p1, p2;
			// absolutely no clue what this does, it just works lol
			for (size_t i = 0, j = (vertices.size() + add) - 1; i < (vertices.size() + add); j = i++)
			{
				p1 = points[i]; p2 = points[j];
				f = (p1.y - first.y) * (p2.x - first.x) - (p2.y - first.y) * (p1.x - first.x);
				twiceArea += f;
				x += (p1.x + p2.x - 2 * first.x) * f;
				y += (p1.y + p2.y - 2 * first.y) * f;
			}
			f = twiceArea * 3;
			delete[] points;
			return Vector2(x / f + first.x, y / f + first.y);
		}
		else
		{
			delete[] points;
			return Vector2::Origin;
		}
	}


	Vector2 Centroid(const Vector2* start, size_t size) noexcept
	{
		Vector2 centroid;
		f64 signedArea = 0.0;
		Vector2 first, second;
		f64 partialSignedArea;
		size_t i;
		for (i = 0; i < size - 1; i++)
		{
			first = start[i];
			second = start[i + 1];
			partialSignedArea = first.Cross(second);
			signedArea += partialSignedArea;
			centroid += (first + second) * partialSignedArea;
		}
		first = start[i];
		second = start[0];
		partialSignedArea = first.Cross(second);
		signedArea += partialSignedArea;
		centroid += (first + second) * partialSignedArea;
		signedArea *= 0.5;
		centroid /= (6.0 * signedArea);
		return centroid;
	}

	i64 Clamp(i64 min, i64 max, i64 x) noexcept
	{
		if (x < min) return min;
		if (x > max) return max;
		return x;
	}

	f64 Clamp(f64 min, f64 max, f64 x) noexcept
	{
		if (x < min) return min;
		if (x > max) return max;
		return x;
	}

	u64 Clamp(u64 min, u64 max, u64 x) noexcept
	{
		if (x < min) return min;
		if (x > max) return max;
		return x;
	}
	
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

	f64 Distance(const Vector& a, const Vector& b) noexcept
	{
		assert(a.GetSize() == b.GetSize());
		f64 dis = 0;
		for (size_t i = 0; i < a.GetSize(); i++)
			dis += SQRD(b[i] - a[i]);
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
		return dis;
	}
	

	f64 DistanceSquared(const Vector& a, const Vector& b) noexcept
	{
		assert(a.GetSize() == b.GetSize());
		f64 dis = 0;
		for (size_t i = 0; i < a.GetSize(); i++)
			dis += SQRD(b[i] - a[i]);
		return dis;
	}

	bool Equal(f64 a, f64 b) noexcept
	{
		return fabs(a - b) <= EPSILON;
	}

	f64 FastInvSqrt(f64 x) noexcept
	{
		f64 y = x;
		const f64 x2 = y * 0.5;
		std::int64_t i = *(std::int64_t*)&y;
		i = 0x5fe6eb50c7b537a9 - (i >> 1);
		y = *(f64*) &i;
		y = y * (1.5 - (x2 * y * y));
		y = y * (1.5 - (x2 * y * y));
		return y;
	}

	f64 FastSqrt(f64 x) noexcept
	{
		f64 y = FastInvSqrt(x);
		return y * x;
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

	f64 Lerp(f64 a, f64 b, f64 t) noexcept
	{
		return a + (b - a) * t;
	}

	i64 Max(i64 a, i64 b) noexcept
	{
		if (a > b)
			return a;
		else
			return b;
	}

	f64 Max(f64 a, f64 b) noexcept
	{
		if (a > b)
			return a;
		else
			return b;
	}

	u64 Max(u64 a, u64 b) noexcept
	{
		if (a > b)
			return a;
		else
			return b;
	}

	i64 Min(i64 a, i64 b) noexcept
	{
		if (a < b)
			return a;
		else
			return b;
	}

	f64 Min(f64 a, f64 b) noexcept
	{
		if (a < b)
			return a;
		else
			return b;
	}

	u64 Min(u64 a, u64 b) noexcept
	{
		if (a < b)
			return a;
		else
			return b;
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
}
