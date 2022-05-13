#include "../../include/physics/Collision/Collision.hpp"
#include "../../include/physics/Collision/CollisionObject.hpp"
#include <iostream>
namespace physics
{
	using namespace serialization;

	CollisionObject::CollisionObject() noexcept
	: Hashable()
	{
		collider.reset(new BoxCollider());
	}

	CollisionObject::CollisionObject(const Collider& c, const Transform& t, const bool& isTrigger) noexcept
	: Hashable(), transform(t), isTrigger(isTrigger), collider(c.Clone())
	{
		classCode = 0x05;
	}

	CollisionObject::CollisionObject(const CollisionObject& c) noexcept
	: Hashable(), transform(c.transform), isTrigger(c.isTrigger), onCollision(c.onCollision), 
	collider(c.GetCollider().Clone())
	{
		classCode = 0x05;
	}

	CollisionObject::CollisionObject(CollisionObject && c) noexcept
	: Hashable(), transform(c.transform), isTrigger(c.isTrigger), onCollision(c.onCollision),
	collider(c.collider.release())
	{
	}

	CollisionObject& CollisionObject::operator=(const CollisionObject& other) noexcept
	{
		if (NotEquals(other))
		{
			transform = other.transform;
			collider.reset(other.GetCollider().Clone());
			isTrigger = other.isTrigger;
			onCollision = other.onCollision;
		}
		return *this;
	}

	CollisionObject* CollisionObject::Clone() const noexcept
	{
		return new CollisionObject(*this);
	}

	CollisionObject::~CollisionObject() noexcept
	{
	}

	bool CollisionObject::Equals(const CollisionObject& other) const noexcept
	{
		if (collider.get())
		{
			return transform.Equals(other.transform) && collider->Equals(other.GetCollider()) && (isTrigger == other.isTrigger);
		}
		return transform.Equals(other.transform) && isTrigger == other.isTrigger;
	}

	bool CollisionObject::IsDynamic() const noexcept
	{
		return _isDynamic;
	}

	std::vector<unsigned char> CollisionObject::GetBytes() const noexcept
	{
		return ToBytes(this, sizeof(*this));
	}

	Collider& CollisionObject::GetCollider() const noexcept
	{
		return *collider;
	}

	int CollisionObject::GetHash() const noexcept
	{
		std::string s = transform.position.ToString() + transform.centerOfRotation.ToString();
		s = s + std::to_string(_isDynamic) + std::to_string(isTrigger);
		int h = 0;
		for (size_t i = 0; i < s.size(); i++)
		{
			h = h * 31 + static_cast<int>(s[i]);
		}
		return h;
	}

	bool CollisionObject::NotEquals(const CollisionObject& other) const noexcept
	{
		if (collider.get())
		{
			return transform.NotEquals(other.transform) ||
				collider->NotEquals(other.GetCollider()) || isTrigger != other.isTrigger;
		}
		return transform.NotEquals(other.transform) || isTrigger != other.isTrigger;
	}

	void CollisionObject::SetCollider(const Collider& c) noexcept
	{
		delete collider.release();
		collider.reset(c.Clone());
	}

	Serializable* CollisionObject::Deserialize(const std::vector<byte>& v,
			const size_t& index, const size_t& length) const
	{
		return NULL;
	}

	unsigned char CollisionObject::GetByte(const size_t& index) const
	{
		return 0x01;
	}

	unsigned long CollisionObject::TotalByteSize() const noexcept
	{
		return 0UL;
	}

	std::vector<unsigned char> CollisionObject::Serialize() const noexcept
	{
		std::vector<unsigned char> vec;
		return vec;
	}
}