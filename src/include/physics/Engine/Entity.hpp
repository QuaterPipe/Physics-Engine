#pragma once
#include "../../SFML/Graphics.hpp"
#include "../Collision/Collision.hpp"
#include "../Collision/Rigidbody.hpp"
#include <memory>

namespace physics
{
	//0x07
	class Entity : public serialization::Serializable, public Hashable
	{
		protected:
			virtual std::vector<unsigned char> GetBytes() const noexcept override;
		public:
			std::unique_ptr<CollisionObject> collider;
			std::string name;
			sf::Sprite sprite;
			Transform transform;
			unsigned short ClassCode = 0x07;
			bool willDraw = true;
			Entity() noexcept;
			Entity(const std::string& name, const CollisionObject& c = Rigidbody(), const sf::Sprite& s = sf::Sprite()) noexcept;
			Entity(const Entity& e) noexcept;
			virtual Entity& operator=(const Entity& other) noexcept;
			virtual ~Entity() noexcept;
			virtual Entity* Clone() const noexcept;
			virtual bool Equals(const Entity& other) const noexcept;
			virtual CollisionObject& GetCollisionObject() const noexcept;
			virtual bool NotEquals(const Entity& other) const noexcept;
			virtual void SetCollisionObject(CollisionObject& c) noexcept;
			virtual void Update() noexcept;
			virtual void FixedUpdate() noexcept;
			virtual Serializable* Deserialize(const std::vector<byte>& v,
			const size_t& index, const size_t& length) const override;
			virtual byte GetByte(const size_t& index) const override;
			virtual unsigned long TotalByteSize() const noexcept override;
			virtual std::vector<byte> Serialize() const noexcept override;
	};
}