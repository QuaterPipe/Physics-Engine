#pragma once
#include "Dynamicbody.hpp"
#include "physics/Collision/BoxCollider.hpp"
#include "physics/Collision/CircleCollider.hpp"
#include "physics/Collision/PolygonCollider.hpp"

namespace physics
{

	class Rigidbody : public Dynamicbody
	{
		public:
			bool isKinematic = false;
			Rigidbody() noexcept;
			Rigidbody(const Collider& c, const Transform& t = Transform(), const bool& isTrigger = false, const PhysicsMaterial& p = PhysicsMaterial(),
				const f64& mass = 1, bool usesGravity = true, const geo::Vector2& drag = geo::Vector2(0.1, 0.1)) noexcept;
			Rigidbody(const Rigidbody& r) noexcept;
			Rigidbody(Rigidbody && r) noexcept;
			bool operator==(const CollisionObject& c) const noexcept override;
			bool operator!=(const CollisionObject& c) const noexcept override;
			~Rigidbody() noexcept;
			Rigidbody& operator=(const Rigidbody& other) noexcept;
			Rigidbody& operator=(Rigidbody&& other) noexcept;
			void ApplyAngularForce(f64 force) noexcept override;
			void ApplyAngularImpulse(f64 force) noexcept override;
			void ApplyForce(const geo::Vector2& Force, const geo::Vector2& contactPoint = geo::Vector2::Infinity) noexcept override;
			void ApplyImpulse(const geo::Vector2& impulse, const geo::Vector2& contactVec = geo::Vector2::Infinity) noexcept override;
			CollisionObject* Clone() const noexcept override;
			f64 ComputeAngularForce(f64 orientation, f64 angVelocity) const noexcept;
			geo::Vector2 ComputeForce(const geo::Vector2& position, const geo::Vector2& velocity) const noexcept override;
			void Move(f64 offsetX, f64 offsetY) noexcept;
			void Update(f64 dt, int rk4step) noexcept override;
			static Rigidbody CreateBox(f64 width, f64 height, f64 x, f64 y, f64 mass, bool isStatic = false) noexcept;
			static Rigidbody CreateCircle(f64 radius, f64 x, f64 y, f64 mass, bool isStatic = false) noexcept;
			static Rigidbody CreatePolygon(size_t nSides, f64 edgeLength, f64 x, f64 y, f64 mass, bool isStatic = false) noexcept;
	};
}
