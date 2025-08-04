#pragma once
#include "BoxCollider.hpp"
#define MAX_POLYGONCOLLIDER_SIZE 100ULL


namespace physics
{
	
	struct PolygonCollider : public Collider
	{
		private:
			Vector2* _points = nullptr;
			Vector2* _normals = nullptr;
			size_t _pointCount;
			Vector2 _center;
			BoxCollider _boundingBox;
			Vector2 _min = Vector2::Infinity;
			Vector2 _max = -Vector2::Infinity;
		public:
			PolygonCollider() noexcept;
			PolygonCollider(const BoxCollider& b) noexcept;
			PolygonCollider(const PolygonCollider& p) noexcept;
			PolygonCollider(PolygonCollider&& p) noexcept;
			PolygonCollider(f64 sideLength, size_t count=3) noexcept;
			PolygonCollider(const Vector2& a, const Vector2& b, const Vector2& c, std::initializer_list<Vector2> extra={}) noexcept;
			PolygonCollider(const std::vector<Vector2>& points);
			~PolygonCollider() noexcept;
			virtual BoxCollider BoundingBox(const Transform& t = Transform()) const noexcept override;
			Collider* Clone() const noexcept override;
			void ComputeMass(f64 density, f64* mass, f64* inertia) const noexcept;
			virtual bool Contains(const Vector2& point, const Transform& t = Transform()) const noexcept override;
			virtual Vector2 GetCenter() const noexcept override;
			bool operator==(const Collider& c) const noexcept override;
			bool operator!=(const Collider& c) const noexcept override;
			PolygonCollider& operator=(const PolygonCollider& p) noexcept;
			PolygonCollider& operator=(PolygonCollider&& p) noexcept;
			Vector2 Max() const noexcept override;
			Vector2 Min() const noexcept override;
			virtual std::vector<Vector2> GetPoints(const Transform& t = Transform()) const noexcept override;
			Vector2 GetNormal(size_t index) const;
			std::vector<Vector2> GetNormals() const noexcept;
			Vector2* GetNormalArray() const noexcept;
			Vector2 GetPoint(size_t index) const;
			size_t GetPointCount() const noexcept;
			Vector2* GetVectorArray() const noexcept;
			void Release() noexcept;
			void Set(const std::vector<Vector2>& points);
			void SetPoint(size_t index, const Vector2& point) noexcept;
			Vector2 SupportPoint(const Vector2& direction) const noexcept;
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
