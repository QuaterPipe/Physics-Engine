#pragma once
#include "physics/Collision/Collision.hpp"
#include "Dynamicbody.hpp"
#include "Spring.hpp"
#define DEFAULT_POINTMASS_RADIUS 0.01
namespace physics
{
	struct Softbody : public Dynamicbody // remember to update softbody copies and comparison operators
	{
		private:
			std::vector<RK4State> _pointStates;
			Vector2 _originalCenter;
			bool _pointsChanged = false;
		public:
			std::vector<PointMass> _originalShape;
			std::vector<PointMass> points;
			std::vector<PointMassSpring> springs;
			size_t pointCount = 0;
			f64 radiusPerPoint = 0.01;
			bool isPressureBody = false;
			bool shapeMatchingOn = true;
			f64 derivedAngle = 0;
			Vector2 derivedPos;
			Vector2 derivedVel;
			PointMassSpring shapeSpring;
			Softbody() noexcept;
			Softbody(const Transform& t, size_t width, size_t height, const PointMassSpring& referenceSpring, const PointMassSpring& shapeSpring, const f64& radiusPerPoint = DEFAULT_POINTMASS_RADIUS, const f64& invMassPerPoint = 1) noexcept;
			Softbody(const Softbody& s) noexcept;
			Softbody(Softbody && s) noexcept;
			Softbody& operator=(const Softbody& s) noexcept;
			Softbody& operator=(Softbody && s) noexcept;
			bool operator==(const CollisionObject& c) const noexcept override;
			bool operator!=(const CollisionObject& c) const noexcept override;
			void ApplyAngularForce(f64 force) noexcept override;
			void ApplyAngularImpulse(f64 force) noexcept override;
			void ApplyForce(const Vector2& Force, const Vector2& contactPoint = Vector2::Infinity) noexcept override;
			void ApplyImpulse(const Vector2& impulse, const Vector2& contactVec = Vector2::Infinity) noexcept override;
			CollisionObject* Clone() const noexcept override;
			f64 ComputeAngularForce(f64 orient, f64 angVelocity) const noexcept;
			Vector2 ComputeForce(const Vector2& position, const Vector2& velocity) const noexcept override;
			void DerivePositionAndAngle() noexcept;
			void FixCollapsing() noexcept;
			PointMass* GetClosestMassPoint(const Vector2& point) const noexcept;
			const std::vector<PointMass>& GetOriginalShape() const noexcept;
			f64 MassScaler() const noexcept override;
			void Update(f64 dt, int rk4step) noexcept override;
			void UpdateCollider() noexcept;
			void UpdateTransform() noexcept;
	};
}