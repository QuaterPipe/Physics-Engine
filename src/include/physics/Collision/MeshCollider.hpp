	#pragma once
#include "BoxCollider.hpp"

namespace physics
{
	//0x05
	struct MeshCollider : public Collider
	{
		protected:
			virtual std::vector<unsigned char> GetBytes() const noexcept override;
		public:
			std::vector<Collider*> colliders;
			MeshCollider() noexcept;
			MeshCollider(const std::vector<Collider*>& colliders) noexcept;
			MeshCollider(const MeshCollider& m) noexcept;
			~MeshCollider() noexcept;
			virtual BoxCollider BoundingBox(const Transform& t = Transform()) const noexcept override;
			Collider* Clone() const noexcept override;
			virtual geo::Vector GetCenter() const noexcept override;
			virtual bool Equals(const MeshCollider& other) const noexcept;
			virtual bool NotEquals(const MeshCollider& other) const noexcept;
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