#pragma once
#include "Collision.hpp"
#include "Dynamicbody.hpp"
namespace physics
{
	struct MassPoint
	{
		geometry::Vector position;
		geometry::Vector velocity;
		geometry::Vector force;
		f64 radius;
		f64 mass = 1;
		MassPoint();
		MassPoint(geometry::Vector position, geometry::Vector velocity, geometry::Vector force, f64 mass, f64 radius = 1) noexcept;
		bool operator==(const MassPoint& other) const noexcept;
		bool operator!=(const MassPoint& other) const noexcept;
	};

	struct Spring
	{
		MassPoint* a = NULL;
		MassPoint* b = NULL;
		f64 stiffness = 0.02;
		f64 restingLength = 1;
		f64 dampingFactor = 0.04;
		f64 ForceExerting() const noexcept;
		bool operator==(const Spring& other) const noexcept;
		bool operator!=(const Spring& other) const noexcept;
	};

	struct Softbody : public Dynamicbody
	{
		private:
			std::vector<PolygonCollider> _colliders;
		public:
			std::vector<std::vector<MassPoint>> points;
			std::vector<Spring> springs;
			unsigned width;
			unsigned height;
			f64 radiusPerPoint;
			bool isPressureBody = false;
			Softbody() noexcept;
			Softbody(const Transform& t, unsigned width, unsigned height, const Spring& spring, const f64& spacing = 1, const f64& radiusPerPoint = 1, const f64& massPerPoint = 1) noexcept;
			Softbody(const Softbody& s) noexcept;
			Softbody(Softbody && s) noexcept;
			virtual Softbody& operator=(const Softbody& s) noexcept;
			virtual Softbody& operator=(Softbody && s) noexcept;
			virtual void ApplyAngularForce(f64 angularVelocity) noexcept override;
			virtual void ApplyForce(const geometry::Vector& force, const geometry::Vector& contactPoint = geometry::Vector::Infinity) noexcept override;
			virtual void ApplyImpulse(const geometry::Vector& impulse, const geometry::Vector& contactVec) noexcept override;
			virtual void ApplySpringForces() noexcept;
			virtual CollisionObject* Clone() const noexcept override;
			virtual bool Equals(const Hashable& other) const noexcept override;
			virtual void FixCollapsing() noexcept;
			virtual bool NotEquals(const Hashable& other) const noexcept override;
			virtual void UpdateCollider() noexcept;
	};
}