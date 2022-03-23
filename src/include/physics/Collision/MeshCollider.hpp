#pragma once
#include "Collider.hpp"

namespace physics
{
	//0x05
	struct MeshCollider : public Collider
	{
		std::vector<Collider*> colliders;
		MeshCollider() noexcept;
		MeshCollider(const std::vector<Collider*>& colliders) noexcept;
		MeshCollider(const MeshCollider& m) noexcept;
		~MeshCollider() noexcept;
		Collider* Clone() const override;
		virtual geometry::Vector GetCenter() const noexcept override;
		geometry::Vector Max() const noexcept override;
		geometry::Vector Min() const noexcept override;
		virtual CollisionPoints TestCollision(
			const Transform& transform,
			const Collider* collider,
			const Transform& colliderTransform) const noexcept override;
		virtual CollisionPoints TestCollision(
			const Transform& transform,
			const CircleCollider* collider,
			const Transform& colliderTransform) const noexcept override;
		virtual CollisionPoints TestCollision(
			const Transform& transform,
			const PolygonCollider* collider,
			const Transform& colliderTransform) const noexcept override;
		virtual CollisionPoints TestCollision(
			const Transform& transform,
			const BoxCollider* collider,
			const Transform& colliderTransform) const noexcept override;
		virtual CollisionPoints TestCollision(
			const Transform& transform,
			const MeshCollider* collider,
			const Transform& colliderTransform) const noexcept override;
		virtual Serializable* Deserialize(const std::vector<byte>& v,
			const size_t& index, const size_t& length) const override;
		virtual byte GetByte(const size_t& index) const override;
		virtual unsigned long TotalByteSize() const noexcept override;
		virtual std::vector<byte> Serialize() const noexcept override;
	
	};
}