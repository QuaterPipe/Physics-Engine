#pragma once
#include "Collider.hpp"
#include "../Engine/Time.hpp"
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
	};

	struct CollisionObject
	{
		protected:
			bool _isDynamic = false;
		public:
			bool isTrigger = false;
			bool isActive = true;
			std::function<void(Collision&, f64)> onCollision;
			std::unique_ptr<Collider> collider;
			Transform transform;
			geo::Vector2& position = transform.position;
			geo::Vector2& centerOfRotation = transform.centerOfRotation;
			geo::Matrix2& scale = transform.scale;
			geo::Matrix2& rotation = transform.rotation;
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