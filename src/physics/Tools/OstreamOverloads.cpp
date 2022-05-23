#include "../../include/physics/Tools/OstreamOverloads.hpp"

namespace physics
{
	std::ostream& operator<<(std::ostream& os, const geo::Vector& v)
	{
		return os << v.ToString();
	}

	std::ostream& operator<<(std::ostream& os, const geo::Line& l)
	{
		return os << l.ToString();
	}

	std::ostream& operator<<(std::ostream& os, const Transform& t)
	{
		os << t.position;
		os << "\n" << t.rotation << "\n";
		return os << t.scale;
	}

	std::ostream& operator<<(std::ostream& os, const geo::Matrix2& m)
	{
		return os << "|" << m.a << " " << m.b << "|\n|" << m.c << " " << m.d << "|";
	}

	std::ostream& operator<<(std::ostream& os, const PolygonCollider& p)
	{
		return os << p.points;
	}

	std::ostream& operator<<(std::ostream& os, const CollisionPoints& c)
	{
		return os << "hasCollision: " << std::boolalpha << c.hasCollision << " c.a: " << c.a << ", c.b: " << c.b << " normal: " << c.normal << " depth: " << c.depth;
	}
}