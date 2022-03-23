#include "../include/physics/Main/Entity.hpp"
#include "../include/physics/Collision/Rigidbody.hpp"
#include <cstring>
namespace physics
{
	using namespace serialization;
	Entity::Entity() noexcept
	{
		_name = "Entity";
		_collider.reset(new Rigidbody());
		_transform = Transform();
	}

	Entity::Entity(const std::string& name, CollisionObject& c, const sf::Sprite& s) noexcept
	{
		_name = name;
		_collider.reset(c.Clone());
		_sprite = s;
		_sprite.setPosition(_transform.position.x, _transform.position.y);
		_transform = _collider->GetTransform();
	}

	Entity::Entity(const Entity& e) noexcept
	{
		_name = e.GetName();
		_collider.reset(e.GetCollisionObject().Clone());
		_sprite = e.GetSprite();
		if (e.GetSprite().getTexture())
			_sprite.setTexture(*e.GetSprite().getTexture());
		_transform = e.GetTransform();
		_sprite.setPosition(_transform.position.x, _transform.position.y);
	}

	Entity::~Entity() noexcept
	{
	}

	Entity& Entity::operator=(const Entity& other) noexcept
	{
		_name = other.GetName();
		_collider.reset(other.GetCollisionObject().Clone());
		_sprite = other.GetSprite();
		if (other.GetSprite().getTexture())
			_sprite.setTexture(*other.GetSprite().getTexture());
		_transform = other.GetTransform();
		_sprite.setPosition(_transform.position.x, _transform.position.y);
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
		if (_collider)
			return (_name == e.GetName()) && _collider->Equals(e.GetCollisionObject()) &&_transform.Equals(e.GetTransform());
		else
			return (_name == e.GetName()) && _transform.Equals(e.GetTransform());	
	}

	CollisionObject& Entity::GetCollisionObject() const noexcept
	{
		return *_collider;
	}

	std::string Entity::GetName() const noexcept
	{
		return _name;
	}

	const sf::Sprite& Entity::GetSprite() const noexcept
	{
		return _sprite;
	}

	Transform Entity::GetTransform() const noexcept
	{
		return _transform;
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
		_collider.reset(c.Clone());
	}

	void Entity::SetName(const std::string& s) noexcept
	{
		_name = s;
	}

	void Entity::SetSprite(const sf::Sprite& s) noexcept
	{
		_sprite = s;
	}

	void Entity::SetTransform(const Transform& t) noexcept
	{
		_transform = t;
		_sprite.setPosition(t.position.x, t.position.y);
		_sprite.setRotation(acos(_transform.rotation.a));
		_collider->SetTransform(t);
	}

	void Entity::Update() noexcept
	{
		_transform = _collider->GetTransform();
		_sprite.setPosition(_transform.position.x, _transform.position.y);
		_sprite.rotate(acos(_transform.rotation.a));
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