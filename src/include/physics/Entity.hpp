#pragma once
#include "../SFML/Graphics.hpp"
#include "Collision.hpp"
#include "CollisionObject.hpp"
#include <memory>

namespace physics
{
	//0x07
	class Entity : public serialization::Serializable, public Hashable
	{
		private:
			std::unique_ptr<CollisionObject> _collider;
			std::string _name;
			sf::Sprite _sprite;
			Transform _transform;
		public:
			unsigned short ClassCode = 0x07;
			Entity() noexcept;
			Entity(const std::string& name, CollisionObject& c, const sf::Sprite& s) noexcept;
			Entity(const Entity& e) noexcept;
			virtual Entity& operator=(const Entity& other) noexcept;
			virtual ~Entity() noexcept;
			virtual Entity* Clone() const noexcept;
			virtual bool Equals(const Hashable& other) const noexcept override;
			virtual CollisionObject& GetCollisionObject() const noexcept;
			virtual std::string GetName() const noexcept;
			virtual const sf::Sprite& GetSprite() const noexcept;
			virtual Transform GetTransform() const noexcept;
			virtual bool NotEquals(const Hashable& other) const noexcept override;
			virtual void SetCollisionObject(CollisionObject& c) noexcept;
			virtual void SetName(const std::string& s) noexcept;
			virtual void SetSprite(const sf::Sprite& s) noexcept;
			virtual void SetTransform(const Transform& t) noexcept;
			virtual void Update() noexcept;
			virtual void FixedUpdate() noexcept;
			virtual Serializable* Deserialize(const std::vector<byte>& v,
			const size_t& index, const size_t& length) const override;
			virtual byte GetByte(const size_t& index) const override;
			virtual unsigned long TotalByteSize() const noexcept override;
			virtual std::vector<byte> Serialize() const noexcept override;
	};
}