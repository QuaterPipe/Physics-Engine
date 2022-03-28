#pragma once
#include "Collider.hpp"
#include <functional>
#include <memory>

namespace physics
{
	struct CollisionObject;
	struct Collision
	{
		CollisionObject* a = NULL;
		CollisionObject* b = NULL;
		CollisionPoints points;
	};

	//0x06
	struct CollisionObject : public serialization::Serializable, public Hashable
	{
		protected:
			bool _isDynamic = false;
		public:
			bool isTrigger = false;
			std::function<void(Collision&, f64)> onCollision;
			std::unique_ptr<Collider> collider;
			Transform transform;
			geometry::Vector& position = transform.position;
			geometry::Vector& centerOfRotation = transform.centerOfRotation;
			geometry::Matrix2& scale = transform.scale;
			geometry::Matrix2& rotation = transform.rotation;
			CollisionObject() noexcept;
			CollisionObject(const Transform& t, const Collider& c, bool isTrigger) noexcept;
			CollisionObject(const CollisionObject& c) noexcept;
			CollisionObject(CollisionObject && c) noexcept;
			virtual ~CollisionObject() noexcept;
			virtual CollisionObject* Clone() const noexcept;
			virtual CollisionObject& operator=(const CollisionObject& other) noexcept;
			virtual bool Equals(const Hashable& other) const noexcept override;
			bool IsDynamic() const noexcept;
			Collider& GetCollider() const noexcept;
			virtual int GetHash() const noexcept;
			bool NotEquals(const Hashable& other) const noexcept override;
			void SetCollider(const Collider& c) noexcept;
			virtual Serializable* Deserialize(const std::vector<byte>& v,
				const size_t& index, const size_t& length) const override;
			virtual byte GetByte(const size_t& index) const override;
			virtual unsigned long TotalByteSize() const noexcept override;
			virtual std::vector<byte> Serialize() const noexcept override;
	};
}