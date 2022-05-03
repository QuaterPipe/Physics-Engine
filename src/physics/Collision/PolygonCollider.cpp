#include "../../include/physics/Collision/Collision.hpp"
namespace physics
{
	using namespace serialization;
	PolygonCollider::PolygonCollider(const PolygonCollider& d) noexcept
	{
		this->pos = d.pos;
		this->points = d.points;
		classCode = 0x04;
	}

	PolygonCollider::PolygonCollider(const geo::Vector& pos,
		f64 distanceBetweenPoints, ulong count	
	) noexcept
	{
		if (distanceBetweenPoints < 0)
			distanceBetweenPoints = fabs(distanceBetweenPoints);
		if (count < 3)
			return;
		f64 angle = 0;
		geo::Vector current(0, 0);
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
		classCode = 0x04;
	}

	PolygonCollider::PolygonCollider(const geo::Vector& pos,
		const geo::Vector& a,
		const geo::Vector& b,
		const geo::Vector& c,
		std::initializer_list<geo::Vector> extra) noexcept
	{
		classCode = 0x04;
		this->pos = pos;
		points = {extra};
		points.insert(points.begin(), c);
		points.insert(points.begin(), b);
		points.insert(points.begin(), a);
	}

	PolygonCollider::~PolygonCollider() noexcept {}

	geo::Vector GetCentroid(std::vector<geo::Vector> points)
	{
		if (points.size())
		{
			geo::Vector first = points.at(0);
			geo::Vector last = points.at(points.size() - 1);
			if (first.x != last.x || first.y != last.y)
			{
				points.push_back(first);
			}
			f64 twiceArea = 0, x = 0, y = 0, f = 0;
			geo::Vector p1, p2;
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
			return geo::Vector(x / f + first.x, y / f + first.y);
		}
		else
			return geo::Vector::Origin;
	}

	geo::Vector PolygonCollider::GetCenter() const noexcept
	{
		return GetCentroid(this->points);
	}

	std::vector<geo::Vector> PolygonCollider::GetPoints(const Transform& t) const noexcept
	{
		std::vector<geo::Vector> v;
		for (geo::Vector vec: points)
			v.push_back(t.TransformVector(vec));
		return v;
	}

	Collider* PolygonCollider::Clone() const
	{
		return new PolygonCollider(*this);
	}

	geo::Vector PolygonCollider::Max() const noexcept
	{
		if (!points.size())
			return geo::Vector::Origin;
		return *std::max(points.begin(), points.end()) + pos;
	}

	geo::Vector PolygonCollider::Min() const noexcept
	{
		if (!points.size())
			return geo::Vector::Origin;
		return *std::min(points.begin(), points.end()) + pos;
	}

	Serializable* PolygonCollider::Deserialize(const std::vector<byte>& v,
			const size_t& index, const size_t& length) const
	{
		return NULL;
	}

	unsigned char PolygonCollider::GetByte(const size_t& index) const
	{
		return 0x01;
	}

	unsigned long PolygonCollider::TotalByteSize() const noexcept
	{
		return 0UL;
	}

	std::vector<unsigned char> PolygonCollider::Serialize() const noexcept
	{
		std::vector<unsigned char> vec;
		return vec;
	}
}