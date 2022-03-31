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

	PointCollider::PointCollider(const geometry::Vector& pos)
	: Collider(), position(pos)
	{
	}

	PointCollider::PointCollider(const PointCollider& p)
	: Collider(), position(p.position)
	{
	}

	Serializable* PointCollider::Deserialize(const std::vector<byte>& v,
			const size_t& index, const size_t& length) const
	{
		return NULL;
	}

	unsigned char PointCollider::GetByte(const size_t& index) const
	{
		return Serialize().at(index);
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