#include "../include/geometry/Math.hpp"


namespace geo
{

	f64 Degrees(const f64& radians) noexcept
	{
		return radians * (180 / M_PI);
	}

	f64 Distance(const Vector &a, const Vector &b) noexcept
	{
		f64 dis = SQRD(b.x - a.x) + SQRD(b.y - a.y);
		return sqrt(dis);
	}

	f64 Distance(const Vector3& a, const Vector3& b) noexcept
	{
		f64 dis = (SQRD(b.x - a.x) + SQRD(b.y - a.y) + SQRD(b.z - a.z));
		return sqrt(dis);
	}

	f64 DistanceSquared(const Vector& a, const Vector& b) noexcept
	{
		f64 dis = (SQRD(b.x - a.x) + SQRD(b.y - a.y));
		return dis;
	}

	f64 DistanceSquared(const Vector3& a, const Vector3& b) noexcept
	{
		f64 dis = (SQRD(b.x - a.x) + SQRD(b.y - a.y) + SQRD(b.z - a.z));
		if (dis < 0) {dis *= -1;}
		return dis;
	}

	f64 Distance(const Line& a, const Vector& b) noexcept
	{
		const Vector ac = b - a.a;
		const Vector ab = a.b - a.a;
		auto project = [&] (const Vector& vA, const Vector& vB) {
			double tmp = vA.Dot(vB) / vB.Dot(vB);
			return Vector(tmp * vB.x, tmp *vB.y);
		};
		const Vector d = project(ac, ab) + a.a;
		const Vector ad = d - a.a;
		auto hypot2 = [&] (const Vector& vA, const Vector& vB) {
			return (vA - vB).Dot(vA - vB);
		};
		const double k = fabs(ab.x) > fabs(ab.y) ? ad.x / ab.x : ad.y / ab.y;
		if (k <= 0)
		{
			return sqrt(hypot2(b, a.a));
		}
		else if (k >= 1)
		{
			return sqrt(hypot2(b, a.b));
		}
		return sqrt(hypot2(b, d));
	}

	f64 FastSqrt(const f64& x) noexcept
	{
		f32 n = x;
	   	static union{int i; float f;} u;
	   	u.i = 0x5F375A86 - (*(int*)&n >> 1);
	   	return (f64)((int(3) - n * u.f * u.f) * n * u.f * 0.5f);
	}

	f64 GetAngle(const Vector &a, const Vector &b, const Vector &c) noexcept
	{
		f64 result = atan2(c.y - b.y, c.x - b.x) - atan2(a.y - b.y, a.x - b.x);
		result = result < 0 ? -result : (M_PI * 2) - result;
		return result;
	}

	f64 GetAngle(const f64& slope) noexcept
	{
		return atan(slope);
	}	

	f64 GetAngle(const Vector& center, const Vector& Vector) noexcept
	{
		return atan2(Vector.y - center.y, Vector.x - center.x);
	}

	Vector GetVectorOnCircle(const Vector& center, const f64& radius, const f64& angle) noexcept
	{
		f64 x = center.x + (fabs(radius) * cos(angle));
		f64 y = center.y + (fabs(radius) < 0 ? -radius : radius * sin(angle));
		return Vector(x, y);
	}

	f64 GetSlope(const Vector& a, const Vector& b) noexcept
	{
		if (a.y == b.y || a.x == b.x)
		{return 0;}
		return (b.y - a.y) / (b.x - a.x);
	}

	bool Intersecting(const Line& a, const Line& b, const bool& isInfLine) noexcept
	{
		return PointOfIntersect(a, b, isInfLine) != Vector::Infinity;
	}

	Vector PointOfIntersect(const Line& a, const Line& b, const bool& isInfLine) noexcept
	{
		auto det = [](std::tuple<f64, f64> a, std::tuple<f64, f64> b) -> f64
		{
			return std::get<0>(a) * std::get<1>(b) - std::get<1>(a) * std::get<0>(b);
		};

		auto xdiff = std::tuple<f64, f64>(a.a().x - a.b().x, b.a().x - b.b().x);
		auto ydiff = std::tuple<f64, f64>(a.a().y - a.b().y, b.a().y - b.b().y);
		f64 div = det(xdiff, ydiff);
		if (div == 0) {return Vector::Infinity;}
		auto d = std::tuple<f64, f64>(det(std::get<0>(a.ToTuple()), std::get<1>(a.ToTuple())), det(std::get<0>(b.ToTuple()), std::get<1>(b.ToTuple())));
		f64 x = det(d, xdiff) / div;
		f64 y = det(d, ydiff) / div;
		Vector p = Vector(x, y);
		if (isInfLine) {return p;}
		if ((Distance(a.a(), p) + Distance(a.b(), p) == a.length()) && (Distance(b.a(), p) + Distance(b.b(), p) == b.length()))
		{
			return p;
		}
		return Vector::Infinity;
		/*auto det = [&](Vector A, Vector B) -> f64 {
			return A.x * B.y - A.y * B.x;
		};
		Vector xdiff = Vector(a.a.x - a.b.x, b.a.x - b.b.x);
		Vector ydiff = Vector(a.a.y - a.b.y, b.a.y - b.b.y);
		f64 div = det(xdiff, ydiff);
		if (fabs(div) < 0.00001)
			return Vector::Infinity;
		Vector d = Vector(det(a.a, a.b), det(b.a, b.b));
		f64 x = det(d, xdiff) / div;
		f64 y = det(d, ydiff) / div;
		Vector p = Vector(x, y);
		if (isInfLine)
			return p;
		if ((DistanceSquared(a.a(), p) + DistanceSquared(a.b(), p) == SQRD(a.length())) && (DistanceSquared(b.a(), p) + DistanceSquared(b.b(), p) == SQRD(b.length())))
		{
			return p;
		}*/
	}

	Vector Centroid(const Vector* start, const Vector* end) noexcept
	{
		
		Vector first = *start;
		Vector last = *end;
		std::vector<Vector> points;
		points.insert(points.end(), start, end);
		if (points.size())
		{
			if (first.x != last.x || first.y != last.y)
			{
				points.push_back(first);
			}
			f64 twiceArea = 0, x = 0, y = 0, f = 0;
			Vector p1, p2;
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
			return Vector(x / f + first.x, y / f + first.y);
		}
		else
			return Vector::Origin;
	}
}