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
			Transform _transform;
			Transform _lastTransform;
			std::unique_ptr<Collider> _collider;
			bool _isTrigger = false;
			bool _isDynamic = false;
			std::function<void(Collision&, f64)> _onCollision;
		public:
			CollisionObject() noexcept;
			CollisionObject(const Transform& t, const Collider& c, bool isTrigger) noexcept;
			CollisionObject(const CollisionObject& c) noexcept;
			virtual ~CollisionObject() noexcept;
			virtual CollisionObject* Clone() const noexcept;
			virtual CollisionObject& operator=(const CollisionObject& other) noexcept;
			virtual bool Equals(const Hashable& other) const noexcept override;
			bool IsTrigger() const noexcept;
			bool IsDynamic() const noexcept;
			Collider& GetCollider() const noexcept;
			virtual int GetHash() const noexcept;
			const geometry::Vector& GetPosition() const noexcept;
			std::function<void(Collision&, f64)> GetOnCollisionFunction() const noexcept;
			const geometry::Matrix2& GetRotation() const noexcept;
			const Transform& GetTransform() const noexcept;
			const Transform& GetLastTransform() const noexcept;
			bool NotEquals(const Hashable& other) const noexcept override;
			void SetCollider(const Collider& c) noexcept;
			void SetIsTrigger(bool b) noexcept;
			void SetLastTransform(const Transform& t) noexcept;
			void SetPosition(const geometry::Vector& p) noexcept;
			void SetOnCollisionFunction(const std::function<void(Collision&, f64)> func) noexcept;
			void SetRotation(const geometry::Matrix2& m) noexcept;
			void SetTransform(const Transform& t) noexcept;
			virtual Serializable* Deserialize(const std::vector<byte>& v,
				const size_t& index, const size_t& length) const override;
			virtual byte GetByte(const size_t& index) const override;
			virtual unsigned long TotalByteSize() const noexcept override;
			virtual std::vector<byte> Serialize() const noexcept override;
	};
}