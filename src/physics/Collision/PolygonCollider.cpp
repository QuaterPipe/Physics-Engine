#include "../../include/physics/Collision/Collision.hpp"
namespace physics
{
	PolygonCollider::PolygonCollider(const BoxCollider& b) noexcept
	: pos(0, 0), points(4)
	{
		points[0] = b.pos;
		points[1] = geo::Vector2(b.x + b.width, b.y);
		points[2] = geo::Vector2(b.x + b.width, b.y + b.height);
		points[3] = geo::Vector2(b.x , b.y + b.height);
	}

	PolygonCollider::PolygonCollider(const PolygonCollider& p) noexcept
	{
		pos = p.pos;
		points = p.points;
	}

	PolygonCollider::PolygonCollider(const geo::Vector2& pos,
		f64 distanceBetweenPoints, ulong count	
	) noexcept
	{
		if (distanceBetweenPoints < 0)
			distanceBetweenPoints = fabs(distanceBetweenPoints);
		if (count < 3)
			return;
		f64 angle = 0;
		geo::Vector2 current(0, 0);
		for (ulong i = 0; i < count; i++)
		{
			points.push_back(current);
			current = geo::GetVectorOnCircle(current, distanceBetweenPoints, angle);
			angle += (count - 2) * M_PI;
			angle = fmod(angle, M_PI * 2);
		}
	}

	PolygonCollider::PolygonCollider()
	{
	}

	PolygonCollider::PolygonCollider(const geo::Vector2& pos,
		const geo::Vector2& a,
		const geo::Vector2& b,
		const geo::Vector2& c,
		std::initializer_list<geo::Vector2> extra) noexcept
	{
		this->pos = pos;
		points = {extra};
		points.insert(points.begin(), c);
		points.insert(points.begin(), b);
		points.insert(points.begin(), a);
	}

	PolygonCollider::~PolygonCollider() noexcept {}

	bool PolygonCollider::operator==(const Collider& c) const noexcept
	{
		if (typeid(c).name() != typeid(*this).name())
			return false;
		return pos == dynamic_cast<const PolygonCollider&>(c).pos && points == dynamic_cast<const PolygonCollider&>(c).points;
	}

	bool PolygonCollider::operator!=(const Collider& c) const noexcept
	{
		if (typeid(c).name() != typeid(*this).name())
			return true;
		return pos != dynamic_cast<const PolygonCollider&>(c).pos || points != dynamic_cast<const PolygonCollider&>(c).points;
	}
	
	BoxCollider PolygonCollider::BoundingBox(const Transform& t) const noexcept
	{
		f64 minx = std::numeric_limits<f64>::max();
		f64 miny = std::numeric_limits<f64>::max();
		f64 maxx = std::numeric_limits<f64>::min();
		f64 maxy = std::numeric_limits<f64>::min();
		for (geo::Vector2 p: points)
		{
			geo::Vector2 tp = t.TransformVector(p);
			minx = tp.x < minx ? tp.x : minx;
			miny = tp.y < miny ? tp.y : miny;
			maxx = tp.x > maxx ? tp.x : maxx;
			maxy = tp.y > maxy ? tp.y : maxy;
		}
		BoxCollider result;
		result.pos.Set(minx, miny);
		result.dimensions.Set(maxx - minx, maxy - miny);
		return result;
	}

	geo::Vector2 GetCentroid(std::vector<geo::Vector2> points)
	{
		if (points.size())
		{
			geo::Vector2 first = points.at(0);
			geo::Vector2 last = points.at(points.size() - 1);
			if (first.x != last.x || first.y != last.y)
			{
				points.push_back(first);
			}
			f64 twiceArea = 0, x = 0, y = 0, f = 0;
			geo::Vector2 p1, p2;
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
			return geo::Vector2(x / f + first.x, y / f + first.y);
		}
		else
			return geo::Vector2::Origin;
	}

	geo::Vector2 PolygonCollider::GetCenter() const noexcept
	{
		return GetCentroid(this->points);
	}

	std::vector<geo::Vector2> PolygonCollider::GetPoints(const Transform& t) const noexcept
	{
		std::vector<geo::Vector2> v;
		for (geo::Vector2 vec: points)
			v.push_back(t.TransformVector(vec));
		return v;
	}

	Collider* PolygonCollider::Clone() const noexcept
	{
		return new PolygonCollider(*this);
	}
	
	geo::Vector2 PolygonCollider::Max() const noexcept
	{
		if (!points.size())
			return geo::Vector2::Origin;
		return *std::max(points.begin(), points.end()) + pos;
	}

	geo::Vector2 PolygonCollider::Min() const noexcept
	{
		if (!points.size())
			return geo::Vector2::Origin;
		return *std::min(points.begin(), points.end()) + pos;
	}
}
