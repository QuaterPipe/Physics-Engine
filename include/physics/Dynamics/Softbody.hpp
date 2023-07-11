#pragma once
#include "physics/Collision/Collision.hpp"
#include "Dynamicbody.hpp"
namespace physics
{
	struct MassPoint
	{
		geo::Vector2 position;
		geo::Vector2 velocity;
		geo::Vector2 force;
		f64 radius;
		f64 invMass = 1;
		MassPoint();
		MassPoint(geo::Vector2 position, geo::Vector2 velocity, geo::Vector2 force, f64 invMass, f64 radius = 1) noexcept;
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
		protected:
			std::vector<PolygonCollider> _colliders;
			bool pointsChanged = false;
		public:
			std::vector<std::vector<MassPoint>> points;
			std::vector<Spring> springs;
			int width;
			int height;
			f64 radiusPerPoint;
			bool isPressureBody = false;
			Softbody() noexcept;
			Softbody(const Transform& t, int width, int height, const Spring& spring, const f64& spacing = 1, const f64& radiusPerPoint = 1, const f64& invMassPerPoint = 1) noexcept;
			Softbody(const Softbody& s) noexcept;
			Softbody(Softbody && s) noexcept;
			virtual Softbody& operator=(const Softbody& s) noexcept;
			virtual Softbody& operator=(Softbody && s) noexcept;
			virtual bool operator==(const CollisionObject& c) const noexcept override;
			virtual bool operator!=(const CollisionObject& c) const noexcept override;
			void ApplyAngularForce(f64 force) noexcept override;
			void ApplyAngularImpulse(f64 force) noexcept override;
			void ApplyForce(const geo::Vector2& Force, const geo::Vector2& contactPoint = geo::Vector2::Infinity) noexcept override;
			void ApplyImpulse(const geo::Vector2& impulse, const geo::Vector2& contactVec = geo::Vector2::Infinity) noexcept override;
			void ApplySpringForces(f64 dt) noexcept;
			virtual CollisionObject* Clone() const noexcept override;
			void FixCollapsing() noexcept;
			virtual void Update(f64 dt) noexcept override;
			void UpdateCollider() noexcept;
	};
}