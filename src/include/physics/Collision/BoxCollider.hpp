#pragma once
#include "Collider.hpp"

namespace physics
{
	
	//0x02
	struct BoxCollider : public Collider
	{
		geometry::Vector pos;
		geometry::Vector dimensions;
		double& x = pos.x;
		double& y = pos.y;
		double& width = dimensions.x;
		double& height = dimensions.y;
		BoxCollider() noexcept;
		BoxCollider(const geometry::Vector& pos, const geometry::Vector& dimensions) noexcept;
		BoxCollider(const BoxCollider& b) noexcept;
		~BoxCollider() noexcept;
		BoxCollider& operator=(const BoxCollider& b);
		Collider* Clone() const noexcept override;
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