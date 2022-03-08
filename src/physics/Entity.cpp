#include "../include/physics/Entity.hpp"
#include "../include/physics/Rigidbody.hpp"
#include <cstring>
namespace physics
{

	Entity::Entity() noexcept
	{
		_name = "Entity";
		_collider.reset(new Rigidbody());
		_transform = Transform();
	}

	Entity::Entity(const std::string& name, CollisionObject& c, const Transform& t, const sf::Sprite& s) noexcept
	{
		_name = name;
		_collider.reset(c.Clone());
		_sprite = s;
		_sprite.setPosition(_transform.position.x, _transform.position.y);
		_transform = t;
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

	Entity::~Entity()
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
	}

	void Entity::Update() noexcept
	{
		_transform = _collider->GetTransform();
		_sprite.setPosition(_transform.position.x, _transform.position.y);
		_sprite.rotate(acos(_transform.rotation.a));
	}
}