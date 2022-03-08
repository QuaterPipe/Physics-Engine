#include "../include/physics/Collision.hpp"

namespace physics
{
	using namespace serialization;
	BoxCollider::BoxCollider() noexcept
	{
		classCode = 0x02;
		pos = geometry::Vector(0, 0);
		dimensions = geometry::Vector(1, 1);
	}

	BoxCollider::BoxCollider(const geometry::Vector& pos, const geometry::Vector& dimensions) noexcept
	{
		classCode = 0x02;
		this->pos = pos;
		this->dimensions = dimensions;
		if (this->dimensions.x < 0) {this->dimensions.x = fabs(this->dimensions.x);}
		if (this->dimensions.y < 0) {this->dimensions.y = fabs(this->dimensions.y);}
	}

	BoxCollider::BoxCollider(const BoxCollider& b) noexcept
	{
		classCode = 0x02;
		this->pos = geometry::Vector(b.pos);
		this->dimensions = geometry::Vector(b.dimensions);
		if (this->dimensions.x < 0) {this->dimensions.x = fabs(this->dimensions.x);}
		if (this->dimensions.y < 0) {this->dimensions.y = fabs(this->dimensions.y);}
		this->x = pos.x;
		this->y = pos.y;
		this->width = dimensions.x;
		this->height = dimensions.y;
	}

	BoxCollider::~BoxCollider() noexcept {}

	Collider* BoxCollider::Clone() const noexcept
	{
		return new BoxCollider(*this);
	}

	geometry::Vector BoxCollider::GetCenterOfMass() const noexcept
	{
		return geometry::Vector(x + width / 2, y + height / 2);
	}

	BoxCollider& BoxCollider::operator=(const BoxCollider& b)
	{
		if (*this != b)
		{
			pos = geometry::Vector(b.pos);
			dimensions = geometry::Vector(b.dimensions);
			x = pos.x;
			y = pos.y;
			width = dimensions.x;
			height = dimensions.y;
		}
		return *this;
	}

	geometry::Vector BoxCollider::Max() const noexcept
	{
		return pos + dimensions;
	}

	geometry::Vector BoxCollider::Min() const noexcept
	{
		return pos;
	}

	Serializable* BoxCollider::Deserialize(const std::vector<byte>& v,
			const size_t& index, const size_t& length) const
	{
		BoxCollider* b = new BoxCollider();
		auto iter = v.begin() + index;
		auto end = v.begin() + index + length;
		writer byteWriter = NULL;
		byteWriter = (writer)&b->pos;
		Archive::WriteBytes(byteWriter, iter, sizeof(b->pos));
		iter += sizeof(b->pos);
		byteWriter = (writer)&b->dimensions;
		Archive::WriteBytes(byteWriter, iter, sizeof(b->dimensions));
		return b;
	}

	unsigned char BoxCollider::GetByte(const size_t& index) const
	{
		return Serialize().at(index);
	}

	unsigned long BoxCollider::TotalByteSize() const noexcept
	{
		return Serialize().size();
	}

	std::vector<unsigned char> BoxCollider::Serialize() const noexcept
	{
		std::vector<unsigned char> bytes;
		reader byteReader = NULL;
#if BIG_ENDIAN
		byteReader = (reader)&pos;
		for (unsigned i = 0; i < sizeof(pos); i++)
			bytes.push_back(byteReader[i]);
		byteReader = (reader)&dimensions;
		for (unsigned i = 0; i < sizeof(dimensions); i++)
			bytes.push_back(byteReader[i]);
#elif SMALL_ENDIAN
		byteReader = (reader)&pos.x;
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