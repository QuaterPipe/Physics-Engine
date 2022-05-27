#pragma once
#include "BoxCollider.hpp"

namespace physics
{
	
	//0x04
	struct PolygonCollider : public Collider
	{
		protected:
			virtual std::vector<unsigned char> GetBytes() const noexcept override;
		public:
			geo::Vector pos;
			std::vector<geo::Vector> points;
			PolygonCollider();
			PolygonCollider(const PolygonCollider& d) noexcept;
			PolygonCollider(const geo::Vector& pos, double distanceBetweenPoints=1, unsigned long count=3) noexcept;
			PolygonCollider(const geo::Vector& pos, const geo::Vector& a, const geo::Vector& b, const geo::Vector& c, std::initializer_list<geo::Vector> extra={}) noexcept;
			~PolygonCollider() noexcept;
			virtual BoxCollider BoundingBox(const Transform& t = Transform()) const noexcept override;
			Collider* Clone() const override;
			virtual geo::Vector GetCenter() const noexcept override;
			virtual bool Equals(const PolygonCollider& other) const noexcept;
			virtual bool NotEquals(const PolygonCollider& other) const noexcept;
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