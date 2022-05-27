#include "../../include/physics/Collision/PointCollider.hpp"

namespace physics
{
	using namespace serialization;
	PointCollider::PointCollider()
	: Collider()
	{
	}

	PointCollider::PointCollider(const f64& x, const f64& y)
	: Collider(), position(x, y)
	{
		
	}

	PointCollider::PointCollider(const geo::Vector& pos)
	: Collider(), position(pos)
	{
	}

	PointCollider::PointCollider(const PointCollider& p)
	: Collider(), position(p.position)
	{
	}

	BoxCollider PointCollider::BoundingBox(const Transform& t) const noexcept
	{
		BoxCollider b;
		b.pos = t.TransformVector(position);
		b.dimensions.Set(1, 1);
		return b;
	}

	Serializable* PointCollider::Deserialize(const std::vector<byte>& v,
			const size_t& index, const size_t& length) const
	{
		return NULL;
	}

	bool PointCollider::Equals(const PointCollider& other) const noexcept
	{
		return position == other.position;
	}

	bool PointCollider::NotEquals(const PointCollider& other) const noexcept
	{
		return position != other.position;
	}

	std::vector<unsigned char> PointCollider::GetBytes() const noexcept
	{
		return ToBytes(this, sizeof(*this));
	}

	unsigned char PointCollider::GetByte(const size_t& index) const
	{
		return Serialize().at(index);
	}

	std::vector<geo::Vector> PointCollider::GetPoints(const Transform& t) const noexcept
	{
		std::vector<geo::Vector> v;
		v.push_back(t.TransformVector(position));
		return v;
	}

	unsigned long PointCollider::TotalByteSize() const noexcept
	{
		return sizeof(*this);
	}

	std::vector<unsigned char> PointCollider::Serialize() const noexcept
	{
		std::vector<unsigned char> vec;
		return vec;
	}
}