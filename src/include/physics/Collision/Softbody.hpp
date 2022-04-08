#pragma once
#include "Collision.hpp"
#include "CollisionObject.hpp"
namespace physics
{
	struct MassPoint
	{
		geometry::Vector position;
		geometry::Vector velocity;
		geometry::Vector force;
		f64 mass = 0;
		MassPoint();
		MassPoint(geometry::Vector position, geometry::Vector velocity, geometry::Vector force, f64 mass) noexcept;
		bool operator==(const MassPoint& other) const noexcept;
		bool operator!=(const MassPoint& other) const noexcept;
	};

	struct Spring
	{
		MassPoint* a = NULL;
		MassPoint* b = NULL;
		f64 stiffness = 1e6;
		f64 restingLength = 0;
		f64 dampingFactor = 1e3;
		f64 ForceExerting() const noexcept;
		bool operator==(const Spring& other) const noexcept;
		bool operator!=(const Spring& other) const noexcept;
	};

	struct Softbody : public CollisionObject
	{
		private:
			std::vector<PolygonCollider> _colliders;
		public:
			std::vector<std::vector<MassPoint>> points;
			std::vector<Spring> springs;
			unsigned width;
			unsigned height;
			bool usesGravity;
			Softbody() noexcept;
			Softbody(const Transform& t, unsigned width, unsigned height, const Spring& spring) noexcept;
			Softbody(const Softbody& s) noexcept;
			Softbody(Softbody && s) noexcept;
			virtual Softbody& operator=(const Softbody& s) noexcept;
			virtual Softbody& operator=(Softbody && s) noexcept;
			virtual void ApplyForce(const geometry::Vector& force, const geometry::Vector& point=geometry::Vector::Infinity) noexcept;
			virtual void ApplySpringForces() noexcept;
			virtual CollisionObject* Clone() const noexcept override;
			virtual bool Equals(const Hashable& other) const noexcept override;
			virtual void FixCollapsing() noexcept;
			virtual bool NotEquals(const Hashable& other) const noexcept override;
			virtual void UpdateCollider() noexcept;
	};
}