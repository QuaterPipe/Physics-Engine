#pragma once
#include "CollisionObject.hpp"

namespace physics
{
	struct PhysicsMaterial
	{
		public:
			f64 staticFriction = 1;
			f64 kineticFriction = 1;
			f64 restitution = 1;
			PhysicsMaterial() noexcept;
			PhysicsMaterial(f64 staticFriction, f64 kineticFriction, f64 restitution) noexcept;
			static const PhysicsMaterial Steel;
			static const PhysicsMaterial Glass;
			static const PhysicsMaterial Ice;
	};

	//0x07
	struct Rigidbody : public CollisionObject
	{
		protected:
			geometry::Vector _gravity = geometry::Vector(0, -9.81);
			geometry::Vector _velocity = geometry::Vector(0, 0);
			geometry::Vector _drag = geometry::Vector(1e-3, 1e-3);
			f64 _mass = 1;
			f64 _invMass = 1;
			f64 _inertia = 1;
			f64 _invInertia = 1;
			PhysicsMaterial _physicsMaterial;
			f64 _angularVelocity = 0;
			f64 _torque = 0;
			bool _usesGravity = true;
			bool _isKinematic = false;
		public:
			Rigidbody() noexcept;
			Rigidbody(const Transform& t, Collider& c, bool isTrigger, f64 mass,
				bool usesGravity=true, f64 staticFriction=0.5, f64 kineticFriction=0.5,
				f64 restitution=0.5) noexcept;
			Rigidbody(const Rigidbody& r) noexcept;
			~Rigidbody() noexcept;
			Rigidbody& operator=(const Rigidbody& other) noexcept;
			void ApplyAngularForce(f64 angularVelocity) noexcept;
			void ApplyForce(const geometry::Vector& force, const geometry::Vector& contactPoint = geometry::Vector::Infinity) noexcept;
			void ApplyImpulse(const geometry::Vector& impulse, const geometry::Vector& contactVec) noexcept;
			CollisionObject* Clone() const noexcept override;
			virtual bool Equals(const Hashable& other) const noexcept override;
			f64 GetAngularVelocity() const noexcept;
			geometry::Vector GetDrag() const noexcept;
			geometry::Vector GetForce() const noexcept;
			geometry::Vector GetGravity() const noexcept;
			f64 GetInertia() const noexcept;
			f64 GetInvInertia() const noexcept;
			f64 GetInvMass() const noexcept;
			f64 GetKineticFriction() const noexcept;
			f64 GetMass() const noexcept;
			PhysicsMaterial GetPhysicsMaterial() const noexcept;
			f64 GetRestitution() const noexcept;
			f64 GetStaticFriction() const noexcept;
			f64 GetTorque() const noexcept;
			geometry::Vector GetVelocity() const noexcept;
			bool IsKinematic() const noexcept;
			void Move(f64 offsetX, f64 offsetY) noexcept;
			virtual bool NotEquals(const Hashable& other) const noexcept override;
			void SetAngularVelocity(f64 angularVelocity) noexcept;
			void SetDrag(const geometry::Vector& drag) noexcept;
			void SetGravity(const geometry::Vector& grav) noexcept;
			void SetInertia(f64 inertia) noexcept;
			void SetIsKinematic(bool isKinematic) noexcept;
			void SetKineticFriction(f64 kineticFriction) noexcept;
			void SetMass(f64 mass) noexcept;
			void SetPhysicsMaterial(const PhysicsMaterial& physicsMaterial) noexcept;
			void SetRestitution(f64 restitution) noexcept;
			void SetStaticFriction(f64 staticFriction) noexcept;
			void SetTorque(f64 torque) noexcept;
			void SetUsesGravity(bool usesGravity) noexcept;
			void SetVelocity(const geometry::Vector& vel) noexcept;
			void Update(f64 dt) const noexcept;
			bool UsesGravity() const noexcept;
			virtual Serializable* Deserialize(const std::vector<byte>& v,
			const size_t& index, const size_t& length) const override;
			virtual byte GetByte(const size_t& index) const override;
			virtual unsigned long TotalByteSize() const noexcept override;
			virtual std::vector<byte> Serialize() const noexcept override;
	};
}