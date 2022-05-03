#pragma once
#include "Collider.hpp"

namespace physics
{
	//0x03
	struct CircleCollider : public Collider
	{
		geo::Vector center = geo::Vector();
		f64 radius = 0;
		CircleCollider() noexcept;
		CircleCollider(const f64& radius) noexcept;
		CircleCollider(geo::Vector center, f64 radius) noexcept;
		CircleCollider(const CircleCollider& c) noexcept;
		~CircleCollider() noexcept;
		Collider* Clone() const override;
		virtual geo::Vector GetCenter() const noexcept override;
		geo::Vector Max() const noexcept override;
		geo::Vector Min() const noexcept override;
		virtual std::vector<geo::Vector> GetPoints(const Transform& t = Transform()) const noexcept override;
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