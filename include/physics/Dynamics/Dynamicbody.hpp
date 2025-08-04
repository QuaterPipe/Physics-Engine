#pragma once
#include "physics/Collision/CollisionObject.hpp"
#define MAX_DYNAMICBODY_CONSTRAINT_COUNT 30

namespace physics
{
	struct Constraint;
	struct RK4Step
	{
		const static int STEP1 = 0;
		const static int STEP2 = 1;
		const static int STEP3 = 2;
		const static int STEP4 = 3;
	};

	struct RK4State
	{
		Vector2 k1X, k2X, k3X, k4X;
		Vector2 k1V, k2V, k3V, k4V;
		Vector2 a1, a2, a3, a4;
		Vector2 tmpX, tmpV;

		RK4State();
		void Reset() noexcept;
	};

	struct Joint;
	struct PhysicsMaterial
	{
		public:
			f64 staticFriction = .3;
			f64 kineticFriction = .1;
			f64 restitution = .5;
			PhysicsMaterial() noexcept;
			PhysicsMaterial(f64 staticFriction, f64 kineticFriction, f64 restitution) noexcept;
			bool operator==(const PhysicsMaterial& other) const noexcept;
			bool operator!=(const PhysicsMaterial& other) const noexcept;
			static const PhysicsMaterial Steel;
			static const PhysicsMaterial Glass;
			static const PhysicsMaterial Ice;
	};

	class Dynamicbody : public CollisionObject
	{
		protected:
			f64 _mass = 1000;
			f64 _invMass = 0.001;
			f64 _inertia = 1000;
			f64 _invInertia = 0.001;
		public:
			Vector2 gravity = Vector2(0, -9.81);
			Vector2 velocity = Vector2(0, 0);
			Vector2 force = Vector2(0, 0);
			Vector2 appliedForce = Vector2(0, 0);
			Vector2 drag = Vector2(0.1, 0.1);
			f64 angularVelocity = 0;
			f64 angularForce = 0;
			f64 appliedAngularForce = 0;
			PhysicsMaterial physicsMaterial;
			bool usesGravity = true;
			bool isStatic = false;
			bool hadCollisionLastFrame = false;
			f64& staticFriction = physicsMaterial.staticFriction;
			f64& kineticFriction = physicsMaterial.kineticFriction;
			f64& restitution = physicsMaterial.restitution;
			RK4State posState;
			RK4State angleState;
			std::vector<Constraint*> constraints;
			Dynamicbody() noexcept;
			Dynamicbody(const Collider& c, const Transform& t = Transform(), bool isTrigger = false,
			const PhysicsMaterial& p = PhysicsMaterial(), f64 mass = 1000, bool usesGravity = true,
			const Vector2& drag = Vector2(0.1, 0.1)) noexcept;
			Dynamicbody(const Dynamicbody& d) noexcept;
			Dynamicbody(Dynamicbody && d) noexcept;
			virtual bool operator==(const CollisionObject& c) const noexcept override;
			virtual bool operator!=(const CollisionObject& c) const noexcept override;
			virtual Dynamicbody& operator=(const Dynamicbody& d) noexcept;
			void AddConstraint(Constraint* constraint) noexcept;
			virtual void ApplyAngularForce(f64 force) noexcept = 0;
			virtual void ApplyAngularImpulse(f64 force) noexcept = 0;
			virtual void ApplyForce(const Vector2& Force, const Vector2& contactPoint = Vector2::Infinity) noexcept = 0;
			virtual void ApplyImpulse(const Vector2& impulse, const Vector2& contactVec = Vector2::Infinity) noexcept = 0;
			virtual Vector2 ComputeForce(const Vector2& position, const Vector2& velocity) const noexcept = 0;
			f64 GetInertia() const noexcept;
			f64 GetInvInertia() const noexcept;
			f64 GetMass() const noexcept;
			f64 GetInvMass() const noexcept;
			f64 KineticEnergy() const noexcept;
			virtual f64 MassScaler() const noexcept;
			void RemoveConstraint(Constraint* constraint) noexcept;
			void SetInertia(f64 inertia) noexcept;
			void SetMass(f64 mass) noexcept;
			virtual void Update(f64 dt, int rk4step) noexcept = 0;
			virtual void Translate(Vector2 offset, Vector2* points, size_t ptCount) noexcept;
	};

	struct Joint
	{
		public:
			Dynamicbody* a = NULL;
			Dynamicbody* b = NULL;
			Vector2 aContact;
			Vector2 bContact;
			Vector2 maxForce = Vector2::Infinity;
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

	struct HingeJoint : public Joint
	{
		public:
			bool locked;
			f64 angularFriction;
			HingeJoint(Dynamicbody* a, Dynamicbody* b, f64 angularFriction) noexcept;
			virtual void Update(f64 dt) noexcept override;
	};
}