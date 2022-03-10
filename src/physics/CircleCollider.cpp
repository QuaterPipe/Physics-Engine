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

	geometry::Vector CircleCollider::GetCenter() const noexcept
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
		auto end = v.begin() + index + length;
		writer byteWriter = NULL;
		// x position.
		byteWriter = (writer)&c->center.x;
		Archive::WriteBytes(byteWriter, iter, sizeof(c->center.x));
		if (iter + sizeof(c->center.x) > end)
		{
			delete c;
			throw std::runtime_error("not enough bytes given for object.");
		}
		iter += sizeof(c->center.x);
		// y position.
		byteWriter = (writer)&c->center.y;
		Archive::WriteBytes(byteWriter, iter, sizeof(c->center.y));
		if (iter + sizeof(c->center.y) > end)
		{
			delete c;
			throw std::runtime_error("not enough bytes given for object.");
		}
		iter += sizeof(c->center.y);
		// radius.
		byteWriter = (writer)&c->radius;
		Archive::WriteBytes(byteWriter, iter, sizeof(c->radius));
		if (iter + sizeof(c->radius) > end)
		{
			delete c;
			throw std::runtime_error("not enough bytes given for object.");
		}
		return c;
	}

	unsigned char CircleCollider::GetByte(const size_t& index) const
	{
		return Serialize().at(index);
	}

	unsigned long CircleCollider::TotalByteSize() const noexcept
	{
		return sizeof(*this);
	}

	std::vector<unsigned char> CircleCollider::Serialize() const noexcept
	{
		std::vector<unsigned char> vec;
		std::vector<unsigned char> bytes = Archive::ReadBytes((reader)&center.x, sizeof(center.x));
		vec.insert(vec.end(), bytes.begin(), bytes.end());
		bytes = Archive::ReadBytes((reader)&center.y, sizeof(center.y));
		vec.insert(vec.end(), bytes.begin(), bytes.end());
		bytes = Archive::ReadBytes((reader)&radius, sizeof(radius));
		vec.insert(vec.end(), bytes.begin(), bytes.end());
		return vec;
	}
}