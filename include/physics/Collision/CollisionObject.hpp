#pragma once
#include "physics/Dynamics/VelocityConstraint.hpp"
#include "Collider.hpp"
#include "physics/Engine/Time.hpp"
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
		// the normal impulse of the collision.
		geo::Vector2 nImp;
		// the friction(tangent) impulse of the collision.
		geo::Vector2 tImp;
		bool persistent = false;
	};

	struct CollisionObject
	{
		protected:
			bool _isDynamic = false;
		public:
			bool isTrigger = false;
			bool isActive = true;
			void (*onCollision) (Collision&, f64) = nullptr;
			std::unique_ptr<Collider> collider;
			Transform transform;
			CollisionObject() noexcept;
			CollisionObject(const Collider& c, const Transform& t = Transform(), const bool& isTrigger = false) noexcept;
			CollisionObject(const CollisionObject& c) noexcept;
			CollisionObject(CollisionObject && c) noexcept;
			virtual bool operator==(const CollisionObject& c) const noexcept;
			virtual bool operator!=(const CollisionObject& c) const noexcept;
			virtual ~CollisionObject() noexcept;
			virtual CollisionObject* Clone() const noexcept;
			virtual CollisionObject& operator=(const CollisionObject& other) noexcept;
			virtual bool IsDynamic() const noexcept;
			Collider& GetCollider() const noexcept;
			int GetHash() const noexcept;
			void SetCollider(const Collider& c) noexcept;
	};
}