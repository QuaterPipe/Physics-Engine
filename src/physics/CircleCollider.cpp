#include "../include/physics/Collision.hpp"

namespace physics
{
	using namespace serialization;
	CircleCollider::CircleCollider(geometry::Vector center, double radius) noexcept
	{
		classCode = 0x03;
		this->center = center;
		this->radius = fabs(radius);
	}

	CircleCollider::CircleCollider() noexcept
	{
		classCode = 0x03;
	}

	CircleCollider::CircleCollider(const CircleCollider& c) noexcept
	{
		classCode = 0x03;
		this->radius = c.radius;
		this->center = c.center;
	}

	CircleCollider::~CircleCollider() noexcept {}

	Collider* CircleCollider::Clone() const
	{
		return new CircleCollider(*this);
	}

	geometry::Vector CircleCollider::GetCenterOfMass() const noexcept
	{
		return center;
	}

	geometry::Vector CircleCollider::Max() const noexcept
	{
		return center + radius;
	}

	geometry::Vector CircleCollider::Min() const noexcept
	{
		return center - radius;
	}

	Serializable* CircleCollider::Deserialize(const std::vector<byte>& v,
			const size_t& index, const size_t& length) const
	{
		CircleCollider* c = new CircleCollider();
		auto iter = v.begin() + index;
		const auto end = v.begin() + index + length;
		writer byteWriter = NULL;
#if BIG_ENDIAN
		byteWriter = (unsigned char*)&c->center;
		for (int i = 0; i < sizeof(c->center); i++)
		{
			byteWriter[i] = iter[i];
			iter++;
			if (iter > end)
				throw std::runtime_error("Not enough bytes for serialization");
		}
		byteWriter = (unsigned char*)&c->radius;
		for (int i = 0; i < sizeof(c->radius); i++)
		{
			byteWriter[i] = iter[i];
			iter++;
			if (iter > end)
				throw std::runtime_error("Not enough bytes for serialization");
		}
#elif SMALL_ENDIAN
		byteWriter = (unsigned char*)&c->center.x;
		for (int i = 0; i < sizeof(c->center.x); i++)
		{
			byteWriter[sizeof(c->center.x) - 1 - i] = iter[i];
			iter++;
			if (iter + sizeof(c->center.x) - 1 > end)
				throw std::runtime_error("Not enough bytes for serialization");
		}
		byteWriter = (unsigned char*)&c->center.y;
		for (int i = 0; i < sizeof(c->center.y); i++)
		{
			byteWriter[sizeof(c->center.y) - 1 - i] = iter[i];
			iter++;
			if (iter + sizeof(c->center.y) - 1 > end)
				throw std::runtime_error("Not enough bytes for serialization");
		}
		byteWriter = (unsigned char*)&c->radius;
		for (int i = 0; i < sizeof(c->radius); i++)
		{
			byteWriter[sizeof(c->radius) - 1 - i] = iter[i];
			iter++;
			if (iter + sizeof(c->radius) - 1 > end)
				throw std::runtime_error("Not enough bytes for serialization");
		}
#endif
		return c;
	}

	unsigned char CircleCollider::GetByte(const size_t& index) const
	{
		return Serialize().at(index);
	}

	unsigned long CircleCollider::TotalByteSize() const noexcept
	{
		return Serialize().size();
	}

	std::vector<unsigned char> CircleCollider::Serialize() const noexcept
	{
		std::vector<unsigned char> bytes;
		reader byteReader = NULL;
#if BIG_ENDIAN
		byteReader = (reader)&center;
		for (unsigned i = 0; i < sizeof(center); i++)
			bytes.push_back(byteReader[i]);
		byteReader = (reader)&radius;
		for (unsigned i = 0; i < sizeof(radius); i++)
			bytes.push_back(byteReader[i]);
#elif SMALL_ENDIAN
		byteReader = (reader)&center.x;
		for (unsigned i = 0; i < sizeof(pos.x); i++)
			bytes.push_back(byteReader[sizeof(pos.x) - 1 - i]);
		byteReader = (reader)&pos.y;
		for (unsigned i = 0; i < sizeof(pos.x); i++)
			bytes.push_back(byteReader[sizeof(pos.y) - 1 - i]);
		byteReader = (reader)&dimensions.x;
		for (unsigned i = 0; i < sizeof(dimensions.x); i++)
			bytes.push_back(byteReader[sizeof(dimensions.x) - 1 - i]);
		byteReader = (reader)&dimensions.y;
		for (unsigned i = 0; i < sizeof(dimensions.y); i++)
			bytes.push_back(byteReader[sizeof(dimensions.y) - 1 - i]);
#endif
		return bytes;
	}
}