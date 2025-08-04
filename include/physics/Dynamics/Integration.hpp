#pragma once
#include "physics/Geometry/Vector.hpp"
namespace physics
{
	struct ForceComputer
	{
		virtual Vector2 ComputeForce(const Vector2& position, const Vector2& velocity) const noexcept = 0;
	};

	void RK4Integrate(Vector2* position, Vector2* velocity, f64 dt, Vector2 (*computeForce)(Vector2, Vector2)) noexcept;
	void RK4Integrate(Vector2* position, Vector2* velocity, f64 dt, const ForceComputer& accel) noexcept;
	void RK4Integrate(f64* position, f64* velocity, f64 dt, f64 (*computeForce)(f64, f64)) noexcept;
	void SymplecticEulerIntegrate(Vector2* position, Vector2* velocity, Vector2* acceleration, f64 dt) noexcept;
	void SymplecticEulerIntegrate(f64* position, f64* velocity, f64* acceleration, f64 dt) noexcept;
}