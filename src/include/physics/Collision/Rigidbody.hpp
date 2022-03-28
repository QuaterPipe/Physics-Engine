#pragma once
#include "CollisionObject.hpp"

namespace physics
{
	struct PhysicsMaterial
	{
		public:
			f64 staticFriction = .1;
			f64 kineticFriction = .1;
			f64 restitution = .1;
			PhysicsMaterial() noexcept;
			PhysicsMaterial(f64 staticFriction, f64 kineticFriction, f64 restitution) noexcept;
			bool operator==(const PhysicsMaterial& other) const noexcept;
			bool operator!=(const PhysicsMaterial& other) const noexcept;
			static const PhysicsMaterial Steel;
			static const PhysicsMaterial Glass;
			static const PhysicsMaterial Ice;
	};

	//0x07
	struct Rigidbody : public CollisionObject
	{
		private:
			f64 _mass = 1;
			f64 _invMass = 1;
			f64 _inertia = 1;
			f64 _invInertia = 1;
		public:
			geometry::Vector gravity = geometry::Vector(0, -9.81);
			geometry::Vector velocity = geometry::Vector(0, 0);
			geometry::Vector drag = geometry::Vector(1e-3, 1e-3);
			f64 angularVelocity = 0;
			f64 torque = 0;
			bool usesGravity = true;
			bool isKinematic = false;
			PhysicsMaterial physicsMaterial;
			double& staticFriction = physicsMaterial.staticFriction;
			double& kineticFriction = physicsMaterial.kineticFriction;
			double& restitution = physicsMaterial.restitution;
			Rigidbody() noexcept;
			Rigidbody(const Transform& t, Collider& c, bool isTrigger, f64 mass,
				bool usesGravity=true, f64 staticFriction=0.5, f64 kineticFriction=0.5,
				f64 restitution=0.5) noexcept;
			Rigidbody(const Rigidbody& r) noexcept;
			Rigidbody(Rigidbody && r) noexcept;
			virtual ~Rigidbody() noexcept;
			Rigidbody& operator=(const Rigidbody& other) noexcept;
			virtual void ApplyAngularForce(f64 angularVelocity) noexcept;
			virtual void ApplyForce(const geometry::Vector& force, const geometry::Vector& contactPoint = geometry::Vector::Infinity) noexcept;
			virtual void ApplyImpulse(const geometry::Vector& impulse, const geometry::Vector& contactVec) noexcept;
			virtual CollisionObject* Clone() const noexcept override;
			virtual bool Equals(const Hashable& other) const noexcept override;
			virtual f64 GetInertia() const noexcept;
			virtual f64 GetInvInertia() const noexcept;
			virtual f64 GetMass() const noexcept;
			virtual f64 GetInvMass() const noexcept;
			virtual void Move(f64 offsetX, f64 offsetY) noexcept;
			virtual bool NotEquals(const Hashable& other) const noexcept override;
			virtual void Update(f64 dt) const noexcept;
			virtual void SetInertia(const f64& inertia) noexcept;
			virtual void SetMass(const f64& mass) noexcept;
			virtual Serializable* Deserialize(const std::vector<byte>& v,
			const size_t& index, const size_t& length) const override;
			virtual byte GetByte(const size_t& index) const override;
			virtual unsigned long TotalByteSize() const noexcept override;
			virtual std::vector<byte> Serialize() const noexcept override;
	};
}