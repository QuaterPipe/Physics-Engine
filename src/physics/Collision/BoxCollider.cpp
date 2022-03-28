#include "../../include/physics/Collision/Collision.hpp"

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

	geometry::Vector BoxCollider::GetCenter() const noexcept
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