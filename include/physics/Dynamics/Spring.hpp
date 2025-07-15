#pragma once
#include "geometry/Vector.hpp"
#include "Integration.hpp"
#define DEFAULT_POINTMASS_RADIUS 0.01
namespace physics
{
	struct PointMass;

	struct Spring
	{
		f64 stiffness = 1e6;
		f64 restingLength = 1;
		f64 dampingFactor = 1e4;
		bool operator==(const Spring& other) const noexcept;
		bool operator!=(const Spring& other) const noexcept;
	};

	struct PointMassSpring : public Spring
	{
		PointMass* a = NULL;
		PointMass* b = NULL;
		size_t aIndex = 0;
		size_t bIndex = 0;
		f64 CalculateForce() const noexcept;
		void ApplyForces() const noexcept;
		geo::Vector2 GetAForceVector(geo::Vector2 aPosition, geo::Vector2 aVelocity) const noexcept;
		geo::Vector2 GetBForceVector(geo::Vector2 bPosition, geo::Vector2 bVelocity) const noexcept;
		f64 PotentialEnergy() const noexcept;
		bool operator==(const PointMassSpring& other) const noexcept;
		bool operator!=(const PointMassSpring& other) const noexcept;
	};

	struct PointMass : public ForceComputer
	{
		public:
			geo::Vector2 position;
			geo::Vector2 velocity;
			geo::Vector2 force;
			f64 radius = DEFAULT_POINTMASS_RADIUS;
			f64 invMass = 1;
			bool correctionOn = false;
			PointMass();
			PointMass(geo::Vector2 position, geo::Vector2 velocity, geo::Vector2 force, f64 invMass, f64 radius = 1) noexcept;
			bool operator==(const PointMass& other) const noexcept;
			bool operator!=(const PointMass& other) const noexcept;
			void AddSpring(const PointMassSpring& spring, bool isA) noexcept;
			void AddCorrectionSpring(const PointMassSpring& spring, bool isA) noexcept;
			geo::Vector2 ComputeForce(const geo::Vector2& Position, const geo::Vector2& Velocity) const noexcept override;
			f64 KineticEnergy() const noexcept;
			bool RemoveSpring(const PointMassSpring& spring) noexcept;
			bool RemoveCorrectionSpring(const PointMassSpring& spring) noexcept;

		private:
			std::vector<PointMassSpring> springs;
			std::vector<bool> springInfo;
			std::vector<PointMassSpring> correctionSprings;
			std::vector<bool> correctionSpringInfo;
	};

	class Rigidbody;
	struct RigidbodySpring
	{
		Rigidbody* a = NULL;
		Rigidbody* b = NULL;
		f64 stiffness = 1e6;
		f64 restingLength = 1;
		f64 dampingFctor = 1e4;
		f64 CalculateForce() const noexcept;
	};
}