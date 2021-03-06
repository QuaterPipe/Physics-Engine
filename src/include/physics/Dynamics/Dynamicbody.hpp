#pragma once
#include "../Collision/CollisionObject.hpp"

namespace physics
{
	struct Joint;
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
			virtual std::vector<unsigned char> GetBytes() const noexcept override;
		public:
			geo::Vector gravity = geo::Vector(0, -9.81);
			geo::Vector velocity = geo::Vector(0, 0);
			geo::Vector drag = geo::Vector(0.1, 0.1);
			geo::Vector force = geo::Vector(0, 0);
			f64 angularVelocity = 0;
			f64 angularForce = 0;
			f64 torque = 0;
			PhysicsMaterial physicsMaterial;
			bool usesGravity = true;
			bool isStatic = false;
			double& staticFriction = physicsMaterial.staticFriction;
			double& kineticFriction = physicsMaterial.kineticFriction;
			double& restitution = physicsMaterial.restitution;
			std::vector<Joint*> joints;
			Dynamicbody() noexcept;
			Dynamicbody(const Collider& c, const Transform& t = Transform(), const bool& isTrigger = false,
			const PhysicsMaterial& p = PhysicsMaterial(), const f64& mass = 1000, bool usesGravity = true,
			const geo::Vector& drag = geo::Vector(0.1, 0.1)) noexcept;
			Dynamicbody(const Dynamicbody& d) noexcept;
			Dynamicbody(Dynamicbody && d) noexcept;
			virtual Dynamicbody& operator=(const Dynamicbody& d) noexcept;
			virtual void ApplyAngularForce(f64 force) noexcept = 0;
			virtual void ApplyForce(const geo::Vector& Force, const geo::Vector& contactPoint = geo::Vector::Infinity) noexcept = 0;
			virtual void ApplyImpulse(const geo::Vector& impulse, const geo::Vector& contactVec = geo::Vector::Infinity) noexcept = 0;
			virtual bool Equals(const Dynamicbody& d) const noexcept;
			f64 GetInertia() const noexcept;
			f64 GetInvInertia() const noexcept;
			f64 GetMass() const noexcept;
			f64 GetInvMass() const noexcept;
			virtual bool NotEquals(const Dynamicbody& d) const noexcept;
			void SetInertia(const f64& inertia) noexcept;
			void SetMass(const f64& mass) noexcept;
			virtual void Update(f64 dt) noexcept;
	};
	struct Joint
	{
		public:
			Dynamicbody* a = NULL;
			Dynamicbody* b = NULL;
			geo::Vector aContact;
			geo::Vector bContact;
			geo::Vector maxForce = geo::Vector::Infinity;
			bool aAndBCollide = false;
			virtual void Update(f64 dt) noexcept = 0;
	};

	struct DistanceJoint : public Joint
	{
		public:
			f64 length;
			DistanceJoint(Dynamicbody* a, Dynamicbody* b, f64 length) noexcept;
			virtual void Update(f64 dt) noexcept override;
	};

	struct SpringJoint : public DistanceJoint
	{
		public:
			f64 stiffness = 2;
			f64 dampingFactor = 0.04;
			SpringJoint(Dynamicbody* a, Dynamicbody* b, f64 length, f64 stiffness = 2, f64 dampingFactor = 0.2) noexcept;
			f64 ForceExerting() const noexcept;
			virtual void Update(f64 dt) noexcept override;		
	};

	struct HingeJoint : public Joint
	{
		public:
			bool locked;
			f64 angularFriction;
			HingeJoint(Dynamicbody* a, Dynamicbody* b, f64 angularFriction) noexcept;
			virtual void Update(f64 dt) noexcept override;
	};
}