#pragma once
#include "Collider.hpp"

namespace physics
{
	
	//0x04
	struct PolygonCollider : public Collider
	{
		geometry::Vector pos;
		std::vector<geometry::Vector> points;
		PolygonCollider();
		PolygonCollider(const PolygonCollider& d) noexcept;
		PolygonCollider(const geometry::Vector& pos, double distanceBetweenPoints=1, unsigned long count=3) noexcept;
		PolygonCollider(const geometry::Vector& pos, const geometry::Vector& a, const geometry::Vector& b, const geometry::Vector& c, std::initializer_list<geometry::Vector> extra={}) noexcept;
		~PolygonCollider() noexcept;
		Collider* Clone() const override;
		virtual geometry::Vector GetCenterOfMass() const noexcept override;
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