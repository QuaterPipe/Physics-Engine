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
				const f64& mass = 1, bool usesGravity = true, const geo::Vector2& drag = geo::Vector2(0.1, 0.1)) noexcept;
			Rigidbody(const Rigidbody& r) noexcept;
			Rigidbody(Rigidbody && r) noexcept;
			virtual bool operator==(const CollisionObject& c) const noexcept override;
			virtual bool operator!=(const CollisionObject& c) const noexcept override;
			virtual ~Rigidbody() noexcept;
			Rigidbody& operator=(const Rigidbody& other) noexcept;
			void ApplyAngularForce(f64 dt, f64 force) noexcept override;
			void ApplyAngularImpulse(f64 dt, f64 force) noexcept override;
			void ApplyForce(f64 dt, const geo::Vector2& Force, const geo::Vector2& contactPoint = geo::Vector2::Infinity) noexcept override;
			void ApplyImpulse(f64 dt, const geo::Vector2& impulse, const geo::Vector2& contactVec = geo::Vector2::Infinity) noexcept override;
			virtual CollisionObject* Clone() const noexcept override;
			void Move(f64 offsetX, f64 offsetY) noexcept;
			virtual void Update(f64 dt) noexcept override;
	};
}
