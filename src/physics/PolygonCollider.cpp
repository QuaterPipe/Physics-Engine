#include "../include/physics/Collision.hpp"
namespace physics
{
	PolygonCollider::PolygonCollider(const PolygonCollider& d) noexcept
	{
		this->pos = d.pos;
		this->points = d.points;
		classCode = 0x04;
	}

	PolygonCollider::PolygonCollider(const geometry::Vector& pos,
		f64 distanceBetweenPoints, ulong count	
	) noexcept
	{
		if (distanceBetweenPoints < 0)
			distanceBetweenPoints = fabs(distanceBetweenPoints);
		if (count < 3)
			return;
		f64 angle = 0;
		geometry::Vector current(0, 0);
		for (ulong i = 0; i < count; i++)
		{
			points.push_back(current);
			current = geometry::GetVectorOnCircle(current, distanceBetweenPoints, angle);
			angle += (count - 2) * M_PI;
			angle = fmod(angle, M_PI * 2);
		}
	}

	PolygonCollider::PolygonCollider()
	{
		classCode = 0x04;
	}

	PolygonCollider::PolygonCollider(const geometry::Vector& pos,
		const geometry::Vector& a,
		const geometry::Vector& b,
		const geometry::Vector& c,
		std::initializer_list<geometry::Vector> extra) noexcept
	{
		classCode = 0x04;
		this->pos = pos;
		points = {extra};
		points.insert(points.begin(), c);
		points.insert(points.begin(), b);
		points.insert(points.begin(), a);
	}

	PolygonCollider::~PolygonCollider() noexcept {}

	geometry::Vector getCentroid(std::vector<geometry::Vector> points)
	{
		if (points.size())
		{
			geometry::Vector first = points.at(0);
			geometry::Vector last = points.at(points.size() - 1);
			if (first.x != last.x || first.y != last.y)
			{
				points.push_back(first);
			}
			f64 twiceArea = 0, x = 0, y = 0, f = 0;
			geometry::Vector p1, p2;
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
			return geometry::Vector(x / f + first.x, y / f + first.y);
		}
		else
			return geometry::Vector::Origin;
	}

	geometry::Vector PolygonCollider::GetCenterOfMass() const noexcept
	{
		return getCentroid(this->points);
	}

	Collider* PolygonCollider::Clone() const
	{
		return new PolygonCollider(*this);
	}

	geometry::Vector PolygonCollider::Max() const noexcept
	{
		if (!points.size())
			return geometry::Vector::Origin;
		return *std::max(points.begin(), points.end()) + pos;
	}

	geometry::Vector PolygonCollider::Min() const noexcept
	{
		if (!points.size())
			return geometry::Vector::Origin;
		return *std::min(points.begin(), points.end()) + pos;
	}

	Serializable* BoxCollider::Deserialize(const std::vector<byte>& v,
			const size_t& index, const size_t& length) const
	{
		BoxCollider* b = new BoxCollider();
		auto iter = v.begin() + index;
		auto end = v.begin() + index + length;
		writer byteWriter = NULL;
		// x position.
		byteWriter = (writer)&b->pos.x;
		Archive::WriteBytes(byteWriter, iter, sizeof(b->pos.x));
		if (iter + sizeof(b->pos.x) > end)
		{
			delete b;
			throw std::runtime_error("not enough bytes given for object.");
		}
		iter += sizeof(b->pos.x);
		// y position.
		byteWriter = (writer)&b->pos.y;
		Archive::WriteBytes(byteWriter, iter, sizeof(b->pos.y));
		if (iter + sizeof(b->pos.y) > end)
		{
			delete b;
			throw std::runtime_error("not enough bytes given for object.");
		}
		iter += sizeof(b->pos.y);
		// x dimension.
		byteWriter = (writer)&b->dimensions.x;
		Archive::WriteBytes(byteWriter, iter, sizeof(b->dimensions.x));
		if (iter + sizeof(b->dimensions.x) > end)
		{
			delete b;
			throw std::runtime_error("not enough bytes given for object.");
		}
		iter += sizeof(b->dimensions.y);
		// y dimension.
		byteWriter = (writer)&b->dimensions.y;
		Archive::WriteBytes(byteWriter, iter, sizeof(b->dimensions.y));
		return b;
	}

	unsigned char BoxCollider::GetByte(const size_t& index) const
	{
		return Serialize().at(index);
	}

	unsigned long BoxCollider::TotalByteSize() const noexcept
	{
		return sizeof(*this);
	}

	std::vector<unsigned char> BoxCollider::Serialize() const noexcept
	{
		std::vector<unsigned char> vec;
		std::vector<unsigned char> bytes = Archive::ReadBytes((reader)&pos.x, sizeof(pos.x));
		vec.insert(vec.end(), bytes.begin(), bytes.end());
		bytes = Archive::ReadBytes((reader)&pos.y, sizeof(pos.y));
		vec.insert(vec.end(), bytes.begin(), bytes.end());
		bytes = Archive::ReadBytes((reader)&dimensions.x, sizeof(dimensions.x));
		vec.insert(vec.end(), bytes.begin(), bytes.end());
		bytes = Archive::ReadBytes((reader)&dimensions.y, sizeof(dimensions.y));
		vec.insert(vec.end(), bytes.begin(), bytes.end());
		return vec;
	}
}