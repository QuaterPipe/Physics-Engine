#include "physics/Collision/Collision.hpp"
#include "physics/Collision/CollisionObject.hpp"
#include <iostream>
namespace physics
{
	CollisionObject::CollisionObject() noexcept
	{
		collider.reset(new BoxCollider());
		transform.SetCOM(collider->GetCenter());
	}

	CollisionObject::CollisionObject(const Collider& c, const Transform& t, const bool& isTrigger) noexcept
	: isTrigger(isTrigger), collider(c.Clone()), transform(t)
	{
		transform.SetCOM(collider->GetCenter());
	}

	CollisionObject::CollisionObject(const CollisionObject& c) noexcept
	: isTrigger(c.isTrigger), onCollision(c.onCollision), 
	collider(c.GetCollider().Clone()), transform(c.transform)
	{
		transform.SetCOM(collider->GetCenter());
	}

	CollisionObject::CollisionObject(CollisionObject && c) noexcept
	: isTrigger(c.isTrigger), onCollision(c.onCollision),
	collider(c.collider.release()), transform(c.transform)
	{
		transform.SetCOM(collider->GetCenter());
	}

	bool CollisionObject::operator!=(const CollisionObject& other) const noexcept
	{
		if (collider.get())
		{
			return transform != (other.transform) ||
				(*collider)==(other.GetCollider()) || isTrigger != other.isTrigger;
		}
		return transform != (other.transform) || isTrigger != other.isTrigger;
	}

	bool CollisionObject::operator==(const CollisionObject& other) const noexcept
	{
		if (collider.get())
		{
			return transform == (other.transform) && (*collider)==(other.GetCollider()) && (isTrigger == other.isTrigger);
		}
		return transform == (other.transform) && isTrigger == other.isTrigger;
	}

	CollisionObject& CollisionObject::operator=(const CollisionObject& other) noexcept
	{
		if ((*this)!=(other))
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

	bool CollisionObject::IsDynamic() const noexcept
	{
		return _isDynamic;
	}

	Collider& CollisionObject::GetCollider() const noexcept
	{
		return *collider;
	}

	int CollisionObject::GetHash() const noexcept
	{
		std::string s = transform.GetPosition().ToString() + transform.GetPosition().ToString();
		s = s + std::to_string(_isDynamic) + std::to_string(isTrigger);
		int h = 0;
		for (size_t i = 0; i < s.size(); i++)
		{
			h = h * 31 + static_cast<int>(s[i]);
		}
		return h;
	}

	void CollisionObject::SetCollider(const Collider& c) noexcept
	{
		delete collider.release();
		collider.reset(c.Clone());
	}
}