#pragma once
#include "Dynamicbody.hpp"

namespace physics
{
	//0x07
	struct Rigidbody : public Dynamicbody
	{
		protected:
			virtual std::vector<unsigned char> GetBytes() const noexcept override;
		public:
			bool isKinematic = false;
			Rigidbody() noexcept;
			Rigidbody(const Collider& c, const Transform& t = Transform(), const bool& isTrigger = false, const PhysicsMaterial& p = PhysicsMaterial(),
				const f64& mass = 1, bool usesGravity = true, const geo::Vector& drag = geo::Vector(0.1, 0.1)) noexcept;
			Rigidbody(const Rigidbody& r) noexcept;
			Rigidbody(Rigidbody && r) noexcept;
			virtual ~Rigidbody() noexcept;
			Rigidbody& operator=(const Rigidbody& other) noexcept;
			virtual void ApplyAngularForce(f64 angularVelocity) noexcept override;
			virtual void ApplyForce(const geo::Vector& Force, const geo::Vector& contactPoint = geo::Vector::Infinity) noexcept override;
			virtual void ApplyImpulse(const geo::Vector& impulse, const geo::Vector& contactVec = geo::Vector::Infinity) noexcept override;
			virtual CollisionObject* Clone() const noexcept override;
			virtual bool Equals(const Rigidbody& other) const noexcept;
			virtual void Move(f64 offsetX, f64 offsetY) noexcept;
			virtual bool NotEquals(const Rigidbody& other) const noexcept;
			virtual Serializable* Deserialize(const std::vector<byte>& v,
			const size_t& index, const size_t& length) const override;
			virtual byte GetByte(const size_t& index) const override;
			virtual unsigned long TotalByteSize() const noexcept override;
			virtual std::vector<byte> Serialize() const noexcept override;
			virtual void Update(f64 dt) noexcept override;
	};
}