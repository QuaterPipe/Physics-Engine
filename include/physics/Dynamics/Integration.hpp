#pragma once
#include "geometry/Vector.hpp"
namespace physics
{
	struct ForceComputer
	{
		virtual geo::Vector2 ComputeForce(const geo::Vector2& position, const geo::Vector2& velocity) const noexcept = 0;
	};

	void RK4Integrate(geo::Vector2* position, geo::Vector2* velocity, f64 dt, geo::Vector2 (*computeForce)(geo::Vector2, geo::Vector2)) noexcept;
	void RK4Integrate(geo::Vector2* position, geo::Vector2* velocity, f64 dt, const ForceComputer& accel) noexcept;
	void RK4Integrate(f64* position, f64* velocity, f64 dt, f64 (*computeForce)(f64, f64)) noexcept;
	void SymplecticEulerIntegrate(geo::Vector2* position, geo::Vector2* velocity, geo::Vector2* acceleration, f64 dt) noexcept;
	void SymplecticEulerIntegrate(f64* position, f64* velocity, f64* acceleration, f64 dt) noexcept;
}