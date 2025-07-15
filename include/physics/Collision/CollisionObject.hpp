#pragma once
#include "Collider.hpp"
#include "physics/Engine/Time.hpp"
#include <functional>
#include <memory>

namespace physics
{
	class CollisionObject;
	struct CollisionManifold
	{
		CollisionObject* a = NULL;
		CollisionObject* b = NULL;
		//geo::Vector2* points = NULL;
		Manifold points;
		size_t size = 0;
		// the normal impulse of the collision.
		geo::Vector2 nImp;
		// the friction(tangent) impulse of the collision.
		geo::Vector2 tImp;
		bool persistent = false;
	};

	class CollisionObject
	{
		protected:
			bool _isDynamic = false;
		public:
			bool isTrigger = false;
			bool isActive = true;
			void (*onCollision) (CollisionManifold&, f64) = nullptr;
			std::unique_ptr<Collider> collider;
			Transform transform;
			int id = 0;
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