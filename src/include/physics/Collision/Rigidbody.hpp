#pragma once
#include "Dynamicbody.hpp"

namespace physics
{
	//0x07
	struct Rigidbody : public Dynamicbody
	{
		public:
			bool isKinematic = false;
			Rigidbody() noexcept;
			Rigidbody(const Collider& c, const Transform& t = Transform(), const bool& isTrigger = false, const PhysicsMaterial& p = PhysicsMaterial(),
				const f64& mass = 1, bool usesGravity = true, const geometry::Vector& drag = geometry::Vector(0.1, 0.1)) noexcept;
			Rigidbody(const Rigidbody& r) noexcept;
			Rigidbody(Rigidbody && r) noexcept;
			virtual ~Rigidbody() noexcept;
			Rigidbody& operator=(const Rigidbody& other) noexcept;
			virtual void ApplyAngularForce(f64 angularVelocity) noexcept override;
			virtual void ApplyForce(const geometry::Vector& Force, const geometry::Vector& contactPoint = geometry::Vector::Infinity) noexcept override;
			virtual void ApplyImpulse(const geometry::Vector& impulse, const geometry::Vector& contactVec) noexcept override;
			virtual CollisionObject* Clone() const noexcept override;
			virtual bool Equals(const Hashable& other) const noexcept override;
			virtual void Move(f64 offsetX, f64 offsetY) noexcept;
			virtual bool NotEquals(const Hashable& other) const noexcept override;
			virtual Serializable* Deserialize(const std::vector<byte>& v,
			const size_t& index, const size_t& length) const override;
			virtual byte GetByte(const size_t& index) const override;
			virtual unsigned long TotalByteSize() const noexcept override;
			virtual std::vector<byte> Serialize() const noexcept override;
			virtual void Update(f64 dt) noexcept override;
	};
}