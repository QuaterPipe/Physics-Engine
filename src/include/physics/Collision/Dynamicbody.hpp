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

	struct Dynamicbody : public CollisionObject
	{
		protected:
			f64 _mass = 1000;
			f64 _invMass = 0.001;
			f64 _inertia = 1000;
			f64 _invInertia = 0.001;
		public:
			geometry::Vector gravity = geometry::Vector(0, -9.81);
			geometry::Vector velocity = geometry::Vector(0, 0);
			geometry::Vector drag = geometry::Vector(0.1, 0.1);
			geometry::Vector force = geometry::Vector(0, 0);
			f64 angularVelocity = 0;
			f64 angularForce = 0;
			f64 torque = 0;
			PhysicsMaterial physicsMaterial;
			bool usesGravity = true;
			bool isStatic = false;
			double& staticFriction = physicsMaterial.staticFriction;
			double& kineticFriction = physicsMaterial.kineticFriction;
			double& restitution = physicsMaterial.restitution;
			Dynamicbody() noexcept;
			Dynamicbody(const Collider& c, const Transform& t = Transform(), const bool& isTrigger = false,
			const PhysicsMaterial& p = PhysicsMaterial(), const f64& mass = 1000, bool usesGravity = true,
			const geometry::Vector& drag = geometry::Vector(0.1, 0.1)) noexcept;
			Dynamicbody(const Dynamicbody& d) noexcept;
			Dynamicbody(Dynamicbody && d) noexcept;
			virtual Dynamicbody& operator=(const Dynamicbody& d) noexcept;
			virtual void ApplyAngularForce(f64 angularVelocity) noexcept = 0;
			virtual void ApplyForce(const geometry::Vector& Force, const geometry::Vector& contactPoint = geometry::Vector::Infinity) noexcept = 0;
			virtual void ApplyImpulse(const geometry::Vector& impulse, const geometry::Vector& contactVec) noexcept = 0;
			virtual f64 GetInertia() const noexcept;
			virtual f64 GetInvInertia() const noexcept;
			virtual f64 GetMass() const noexcept;
			virtual f64 GetInvMass() const noexcept;
			virtual void SetInertia(const f64& inertia) noexcept;
			virtual void SetMass(const f64& mass) noexcept;
			virtual void Update(f64 dt) noexcept;
	};
}