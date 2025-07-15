#pragma once
#include "BoxCollider.hpp"
#define MAX_POLYGONCOLLIDER_SIZE 100ULL


namespace physics
{
	
	struct PolygonCollider : public Collider
	{
		private:
			geo::Vector2* _points = nullptr;
			geo::Vector2* _normals = nullptr;
			size_t _pointCount;
			geo::Vector2 _center;
			BoxCollider _boundingBox;
			geo::Vector2 _min = geo::Vector2::Infinity;
			geo::Vector2 _max = -geo::Vector2::Infinity;
		public:
			PolygonCollider() noexcept;
			PolygonCollider(const BoxCollider& b) noexcept;
			PolygonCollider(const PolygonCollider& p) noexcept;
			PolygonCollider(PolygonCollider&& p) noexcept;
			PolygonCollider(f64 sideLength, size_t count=3) noexcept;
			PolygonCollider(const geo::Vector2& a, const geo::Vector2& b, const geo::Vector2& c, std::initializer_list<geo::Vector2> extra={}) noexcept;
			PolygonCollider(const std::vector<geo::Vector2>& points);
			~PolygonCollider() noexcept;
			virtual BoxCollider BoundingBox(const Transform& t = Transform()) const noexcept override;
			Collider* Clone() const noexcept override;
			void ComputeMass(f64 density, f64* mass, f64* inertia) const noexcept;
			virtual bool Contains(const geo::Vector2& point, const Transform& t = Transform()) const noexcept override;
			virtual geo::Vector2 GetCenter() const noexcept override;
			bool operator==(const Collider& c) const noexcept override;
			bool operator!=(const Collider& c) const noexcept override;
			PolygonCollider& operator=(const PolygonCollider& p) noexcept;
			PolygonCollider& operator=(PolygonCollider&& p) noexcept;
			geo::Vector2 Max() const noexcept override;
			geo::Vector2 Min() const noexcept override;
			virtual std::vector<geo::Vector2> GetPoints(const Transform& t = Transform()) const noexcept override;
			geo::Vector2 GetNormal(size_t index) const;
			std::vector<geo::Vector2> GetNormals() const noexcept;
			geo::Vector2* GetNormalArray() const noexcept;
			geo::Vector2 GetPoint(size_t index) const;
			size_t GetPointCount() const noexcept;
			geo::Vector2* GetVectorArray() const noexcept;
			void Release() noexcept;
			void Set(const std::vector<geo::Vector2>& points);
			void SetPoint(size_t index, const geo::Vector2& point) noexcept;
			geo::Vector2 SupportPoint(const geo::Vector2& direction) const noexcept;
			virtual Manifold TestCollision(
				const Transform& transform,
				const Collider* collider,
				const Transform& colliderTransform) const noexcept override;
			virtual Manifold TestCollision(
				const Transform& transform,
				const CircleCollider* collider,
				const Transform& colliderTransform) const noexcept override;
			virtual Manifold TestCollision(
				const Transform& transform,
				const PolygonCollider* collider,
				const Transform& colliderTransform) const noexcept override;
			virtual Manifold TestCollision(
				const Transform& transform,
				const BoxCollider* collider,
				const Transform& colliderTransform) const noexcept override;
			virtual Manifold TestCollision(
				const Transform& transform,
				const MeshCollider* collider,
				const Transform& colliderTransform) const noexcept override;
	};
}
