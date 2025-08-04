#pragma once
#include "Dynamicbody.hpp"
#include "Spring.hpp"
#include "../Collision/PolygonCollider.hpp"
namespace physics
{
	class Pressurebody : public Dynamicbody
	{
		private:
			f64 _radius = 0;
			f64 _pressureScalar = 0;
			size_t _pointCount = 0;
			std::vector<PointMass> _points;
			std::vector<PointMassSpring> _springs;
			std::vector<Vector2> _pressureForces;
			std::vector<RK4State> _pointStates;
			void _UpdatePressureForces(int rk4step) noexcept;
		public:
			Pressurebody() noexcept;
			Pressurebody(f64 pressure, f64 radius, size_t pointCount, f64 mass, const PointMassSpring& sideSpring) noexcept;
			Pressurebody(const Pressurebody& p) noexcept;
			Pressurebody(Pressurebody&& p) noexcept;
			Pressurebody& operator=(const Pressurebody& p) noexcept;
			Pressurebody& operator=(Pressurebody&& p) noexcept;
			bool operator==(const CollisionObject& other) const noexcept override;
			bool operator!=(const CollisionObject& other) const noexcept override;
			void ApplyAngularForce(f64 Force) noexcept override;
			void ApplyAngularImpulse(f64 impulse) noexcept override;
			void ApplyForce(const Vector2& Force, const Vector2& contactPoint = Vector2::Infinity) noexcept override;
			void ApplyImpulse(const Vector2& impulse, const Vector2& contactVec = Vector2::Infinity) noexcept override;
			CollisionObject* Clone() const noexcept override;
			Vector2 ComputeForce(const Vector2& position, const Vector2& Velocity) const noexcept override;
			void DerivePositionAndAngle() noexcept;
			f64 GetRadius() const noexcept;
			const std::vector<PointMass>& GetPoints() const noexcept;
			f64 GetPressureScalar() const noexcept;
			const std::vector<PointMassSpring>& GetSprings() const noexcept;
			f64 GetVolume(int rk4step = -1) const noexcept;
			f64 MassScaler() const noexcept override;
			void Update(f64 dt, int rk4step) noexcept override;
			void UpdateCollider() noexcept;
			void UpdateTransform() noexcept;
			virtual void Translate(Vector2 offset, Vector2* points, size_t ptCount) noexcept override;
	};
}