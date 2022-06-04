#pragma once
#include "Collider.hpp"

namespace physics
{
	
	//0x02
	struct BoxCollider : public Collider
	{
		protected:
			virtual std::vector<unsigned char> GetBytes() const noexcept override;
		public:
			geo::Vector pos;
			geo::Vector dimensions;
			double& x = pos.x;
			double& y = pos.y;
			double& width = dimensions.x;
			double& height = dimensions.y;
			BoxCollider() noexcept;
			BoxCollider(const f64& width, const f64& height) noexcept;
			BoxCollider(const geo::Vector& pos, const geo::Vector& dimensions) noexcept;
			BoxCollider(const BoxCollider& b) noexcept;
			~BoxCollider() noexcept;
			BoxCollider& operator=(const BoxCollider& b);
			virtual BoxCollider BoundingBox(const Transform& t = Transform()) const noexcept override;
			Collider* Clone() const noexcept override;
			virtual geo::Vector GetCenter() const noexcept override;
			virtual std::vector<geo::Vector> GetPoints(const Transform& t = Transform()) const noexcept override;
			virtual bool Equals(const BoxCollider& other) const noexcept;
			virtual bool NotEquals(const BoxCollider& other) const noexcept;
			geo::Vector Max() const noexcept override;
			geo::Vector Min() const noexcept override;
			bool Overlaps(const BoxCollider& b) const noexcept;
			sf::RectangleShape ToShape() const noexcept;
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