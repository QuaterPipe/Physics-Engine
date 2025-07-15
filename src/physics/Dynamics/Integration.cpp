#include "physics/Dynamics/Integration.hpp"

namespace physics
{

	void RK4Integrate(f64* position, f64* velocity, f64 dt, f64 (*computeForce)(f64, f64)) noexcept
	{
		if (!position || !velocity)
			return;
		f64 k1X, k2X, k3X, k4X;
		f64 k1V, k2V, k3V, k4V;
		f64 a1, a2, a3, a4;
		f64 x2, x3, x4;
		f64 v2, v3, v4;
		a1 = computeForce(*position, *velocity);
		k1X = a1;
		k1V = *velocity;

		x2 = *position + 0.5 * dt * k1X;
		v2 = *velocity + 0.5 * dt * k1V;
		a2 = computeForce(x2, v2);
		k2X = v2;
		k2V = a2;

		//k3
		x3 = *position + 0.5 * dt * k2X;
		v3 = *velocity + 0.5 * dt * k2V;
		a3 = computeForce(x3, v3);
		k3X = v3;
		k3V = a3;

		//k4
		x4 = *position + dt * k3X;
		v4 = *velocity + dt * k3V;
		a4 = computeForce(x4, v4);
		k4X = v4;
		k4V = a4;

		*position = *position + (dt / 6.0) * (k1X + 2 * k2X + 2 * k3X + k4X);
		*velocity = *velocity + (dt / 6.0) * (k1V + 2 * k2V + 2 * k3V + k4V);
	}

	void RK4Integrate(geo::Vector2* position, geo::Vector2* velocity, f64 dt, geo::Vector2(*computeForce)(geo::Vector2, geo::Vector2)) noexcept
	{
		if (!position || !velocity)
			return;
		geo::Vector2 k1X, k2X, k3X, k4X;
		geo::Vector2 k1V, k2V, k3V, k4V;
		geo::Vector2 a1, a2, a3, a4;
		geo::Vector2 x2, x3, x4;
		geo::Vector2 v2, v3, v4;
		a1 = computeForce(*position, *velocity);
		k1X = a1;
		k1V = *velocity;

		x2 = *position + 0.5 * dt * k1X;
		v2 = *velocity + 0.5 * dt * k1V;
		a2 = computeForce(x2, v2);
		k2X = v2;
		k2V = a2;

		//k3
		x3 = *position + 0.5 * dt * k2X;
		v3 = *velocity + 0.5 * dt * k2V;
		a3 = computeForce(x3, v3);
		k3X = v3;
		k3X = a3;

		//k4
		x4 = *position + dt * k3X;
		v4 = *velocity + dt * k3V;
		a4 = computeForce(x4, v4);
		k4X = v4;
		k4V = a4;

		*position = *position + (dt / 6.0) * (k1X + 2 * k2X + 2 * k3X + k4X);
		*velocity = *velocity + (dt / 6.0) * (k1V + 2 * k2V + 2 * k3V + k4V);
	}

	void RK4Integrate(geo::Vector2* position, geo::Vector2* velocity, f64 dt, const ForceComputer& accel) noexcept
	{
		if (!position || !velocity)
			return;
			geo::Vector2 k1X, k2X, k3X, k4X;
			geo::Vector2 k1V, k2V, k3V, k4V;
			geo::Vector2 a1, a2, a3, a4;
			geo::Vector2 x2, x3, x4;
			geo::Vector2 v2, v3, v4;
		
		//k1
		a1 = accel.ComputeForce(*position, *velocity);
		k1X = a1;
		k1V = *velocity;

		//k2
		x2 = *position  + 0.5 * dt * k1X;
		v2 = *velocity + 0.5 * dt * k1V;
		a2 = accel.ComputeForce(x2, v2);
		k2X = v2;
		k2V = a2;

		//k3
		x3 = *position + 0.5 * dt * k2X;
		v3 = *velocity + 0.5 * dt * k2V;
		a3 = accel.ComputeForce(x3, v3);
		k3X = v3;
		k3V = a3;

		//k4
		x4 = *position + dt * k3X;
		v4 = *velocity + dt * k3V;
		a4 = accel.ComputeForce(x4, v4);
		k4X = v4;
		k4V = a4;

		*position = *position + (dt / 6.0) * (k1X + 2 * k2X + 2 * k3X + k4X);
		*velocity = *velocity + (dt / 6.0) * (k1V + 2 * k2V + 2 * k3V + k4V);
	}

	void SymplecticEulerIntegrate(f64* position, f64* velocity, f64* acceleration, f64 dt) noexcept
	{
		if (!position || !velocity || !acceleration)
			return;
		*velocity += *acceleration * dt;
		*position += *velocity * dt;
	}


	void SymplecticEulerIntegrate(geo::Vector2* position, geo::Vector2* velocity, geo::Vector2* acceleration, f64 dt) noexcept
	{
		if (!position || !velocity || !acceleration)
			return;
		*velocity += *acceleration * dt;
		*position += *velocity * dt;
	}
}