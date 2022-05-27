#include "../../include/physics/Collision/Collision.hpp"

namespace physics
{
	using namespace serialization;
	BoxCollider::BoxCollider() noexcept
	: pos(0, 0), dimensions(1, 1)
	{
		classCode = 0x02;
	}

	BoxCollider::BoxCollider(const f64& width, const f64& height) noexcept
	: pos(0, 0), dimensions(width, height)
	{
	}

	BoxCollider::BoxCollider(const geo::Vector& pos, const geo::Vector& dimensions) noexcept
	: pos(pos), dimensions(dimensions.Abs())
	{
		classCode = 0x02;
	}

	BoxCollider::BoxCollider(const BoxCollider& b) noexcept
	: pos(b.pos), dimensions(b.dimensions.Abs())
	{
		classCode = 0x02;
	}

	BoxCollider::~BoxCollider() noexcept
	{
	}

	BoxCollider BoxCollider::BoundingBox(const Transform& t) const noexcept
	{
		PolygonCollider p(geo::Vector(0, 0), pos, geo::Vector(x + width, y), pos + dimensions, {geo::Vector(x, y + height)});
		return p.BoundingBox(t);
	}

	Collider* BoxCollider::Clone() const noexcept
	{
		return new BoxCollider(*this);
	}

	BoxCollider& BoxCollider::operator=(const BoxCollider& b)
	{
		if (*this != b)
		{
			pos = geo::Vector(b.pos);
			dimensions = geo::Vector(b.dimensions);
			x = pos.x;
			y = pos.y;
			width = dimensions.x;
			height = dimensions.y;
		}
		return *this;
	}

	bool BoxCollider::Equals(const BoxCollider& other) const noexcept
	{
		return pos == other.pos && dimensions == other.dimensions;
	}

	bool BoxCollider::NotEquals(const BoxCollider& other) const noexcept
	{
		return pos != other.pos || dimensions != other.dimensions;
	}	
	
	geo::Vector BoxCollider::GetCenter() const noexcept
	{
		return pos + dimensions / 2;
	}

	std::vector<unsigned char> BoxCollider::GetBytes() const noexcept
	{
		return ToBytes(this, sizeof(*this));
	}

	geo::Vector BoxCollider::Max() const noexcept
	{
		return pos + dimensions;
	}

	geo::Vector BoxCollider::Min() const noexcept
	{
		return pos;
	}

	bool BoxCollider::Overlaps(const BoxCollider& b) const noexcept
	{
		auto numInRange = [&] (double value, double minVal, double maxVal){
			return (value >= minVal) && (value <= maxVal);
		};
		bool xOverlaps = numInRange(x, b.x, b.x + b.width) ||
			numInRange(b.x, x, x + width);
		bool yOverlaps = numInRange(y, b.y, b.y + b.height) ||
			numInRange(b.y, y, y + height);
		return xOverlaps && yOverlaps;
	}

	std::vector<geo::Vector> BoxCollider::GetPoints(const Transform& t) const noexcept
	{
		std::vector<geo::Vector> v;
		v.push_back(t.TransformVector(pos));
		v.push_back(t.TransformVector(geo::Vector(x + width, y)));
		v.push_back(t.TransformVector(pos + dimensions));
		v.push_back(t.TransformVector(geo::Vector(x, y + height)));
		return v;
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