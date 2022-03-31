#pragma once
#include "Collider.hpp"
namespace physics
{
	struct PointCollider : public Collider
	{
		geometry::Vector position;
		PointCollider();
		PointCollider(const f64& x, const f64& y);
		PointCollider(const geometry::Vector& pos);
		PointCollider(const PointCollider& p);
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
		virtual CollisionPoints TestCollision(
			const Transform& transform,
			const PointCollider* collider,
			const Transform& colliderTransform) const noexcept override;
		virtual Serializable* Deserialize(const std::vector<byte>& v,
			const size_t& index, const size_t& length) const override;
		virtual byte GetByte(const size_t& index) const override;
		virtual unsigned long TotalByteSize() const noexcept override;
		virtual std::vector<byte> Serialize() const noexcept override;
	};
}