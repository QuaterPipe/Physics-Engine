#include "../../include/physics/Engine/Entity.hpp"
#include "../../include/physics/Collision/Rigidbody.hpp"
#include <cstring>
namespace physics
{
	using namespace serialization;
	Entity::Entity() noexcept
	{
		name = "Entity";
		collider.reset(new Rigidbody());
		transform = Transform();
	}

	Entity::Entity(const std::string& name, CollisionObject& c, const sf::Sprite& s) noexcept
	: name(name), sprite(s), transform(c.transform)
	{
		collider.reset(c.Clone());
		sprite.setPosition(transform.position.x, transform.position.y);
	}

	Entity::Entity(const Entity& e) noexcept
	: name(e.name), sprite(e.sprite), transform(e.transform)
	{
		collider.reset(e.GetCollisionObject().Clone());
		if (e.sprite.getTexture())
			sprite.setTexture(*e.sprite.getTexture());
		sprite.setPosition(transform.position.x, transform.position.y);
	}

	Entity::~Entity() noexcept
	{
	}

	Entity& Entity::operator=(const Entity& other) noexcept
	{
		name = other.name;
		collider.reset(other.GetCollisionObject().Clone());
		sprite = other.sprite;
		if (other.sprite.getTexture())
			sprite.setTexture(*other.sprite.getTexture());
		transform = other.transform;
		sprite.setPosition(transform.position.x, transform.position.y);
		return *this;
	}

	Entity* Entity::Clone() const noexcept
	{
		return new Entity(*this);
	}

	bool Entity::Equals(const Hashable& other) const noexcept
	{
		Entity e(*this);
		try
		{
			e = dynamic_cast<const Entity&>(other);
		}
		catch(const std::bad_cast& e)
		{
			return false;
		}
		if (collider)
			return (name == e.name) && collider->Equals(e.GetCollisionObject()) &&transform.Equals(e.transform);
		else
			return (name == e.name) && transform.Equals(e.transform);	
	}

	CollisionObject& Entity::GetCollisionObject() const noexcept
	{
		return *collider;
	}

	void Entity::FixedUpdate() noexcept
	{
	}

	bool Entity::NotEquals(const Hashable& other) const noexcept
	{
		return !Equals(other);
	}

	void Entity::SetCollisionObject(CollisionObject& c) noexcept
	{
		collider.reset(c.Clone());
	}

	void Entity::Update() noexcept
	{
		transform = collider->transform;
		sprite.setPosition(transform.position.x, transform.position.y);
		sprite.rotate(acos(transform.rotation.a));
	}

	Serializable* Entity::Deserialize(const std::vector<byte>& v,
		const size_t& index, const size_t& length) const
	{
		return NULL;
	}

	unsigned char Entity::GetByte(const size_t& index) const
	{
		return 0x01;
	}

	unsigned long Entity::TotalByteSize() const noexcept
	{
		return 0UL;
	}

	std::vector<unsigned char> Entity::Serialize() const noexcept
	{
		std::vector<unsigned char> vec;
		return vec;
	}
}