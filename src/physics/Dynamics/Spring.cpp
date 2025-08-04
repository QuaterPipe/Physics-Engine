#include "physics/Dynamics/Spring.hpp"
#include "physics/Geometry/Math.hpp"
#include "physics/Geometry/Vector.hpp"
#include <iostream>

namespace physics
{

	bool Spring::operator==(const Spring& other) const noexcept
	{
		return dampingFactor == other.dampingFactor && restingLength == other.restingLength &&
			stiffness == other.stiffness;
	}

	bool Spring::operator!=(const Spring& other) const noexcept
	{
		return dampingFactor != other.dampingFactor || restingLength != other.restingLength ||
			stiffness != other.stiffness;
	}

	f64 PointMassSpring::CalculateForce() const noexcept
	{
		if (!a || !b)
			return Vector2::Infinity.x;
		f64 dist = Distance(a->position, b->position);
		if (dist == 0)
			return 0;
		f64 Fs = (dist - restingLength) * stiffness;
		f64 Fd = -(b->position - a->position).Normalized().Dot(b->velocity - a->velocity) * dampingFactor;
		return Fs + Fd;
	}

	bool PointMassSpring::operator==(const PointMassSpring& other) const noexcept
	{
		return restingLength == other.restingLength && dampingFactor == other.dampingFactor &&
			stiffness == other.stiffness;
	}

	bool PointMassSpring::operator!=(const PointMassSpring& other) const noexcept
	{
		return restingLength != other.restingLength || dampingFactor != other.dampingFactor ||
			stiffness != other.stiffness;
	}

	void PointMassSpring::ApplyForces() const noexcept
	{
		if (!a || !b)
			return;
		f64 force = CalculateForce();
		a->force += force * (b->position - a->position).Normalized();
		b->force += force * (a->position - b->position).Normalized();
	}

	Vector2 PointMassSpring::GetAForceVector(Vector2 aPosition, Vector2 aVelocity) const noexcept
	{
		if (!a || !b)
			return Vector2::Infinity;
		Vector2 delta = b->position - a->position;
		f64 dist = delta.GetMagnitudeExact();
		if (dist == 0.0)
			return Vector2(0, 0);
		Vector2 dir = delta / dist;
		f64 Fs = (dist - restingLength) * stiffness;
		f64 Fd = -dir.Dot(b->velocity - aVelocity) * dampingFactor;
		return (Fs - Fd) * dir;
	}

	Vector2 PointMassSpring::GetBForceVector(Vector2 bPosition, Vector2 bVelocity) const noexcept
	{
		if (!a || !b)
			return Vector2::Infinity;
		Vector2 delta = a->position - bPosition;
		f64 dist = delta.GetMagnitudeExact();
		if (dist == 0.0)
			return Vector2(0, 0);
		Vector2 dir = delta / dist;
		f64 Fs = (dist - restingLength) * stiffness;
		f64 Fd = -dir.Dot(a->velocity - bVelocity) * dampingFactor;
		return (Fs - Fd) * dir;
	}

	f64 PointMassSpring::PotentialEnergy() const noexcept
	{
		if (!a || !b)
			return Vector2::Infinity.x;
		f64 stretch = (b->position - a->position).GetMagnitudeExact() - restingLength;
		return 0.5 * stiffness * SQRD(stretch);
	}

	PointMass::PointMass()
	{
	}

	PointMass::PointMass(Vector2 position, Vector2 velocity, Vector2 force, f64 invMass, f64 radius) noexcept
		: position(position), velocity(velocity), force(force), radius(radius), invMass(invMass)
	{
	}

	bool PointMass::operator==(const PointMass& other) const noexcept
	{
		return position == other.position && velocity == other.velocity &&
			force == other.force && invMass == other.invMass && radius == other.radius &&
			correctionOn == other.correctionOn;
	}

	bool PointMass::operator!=(const PointMass& other) const noexcept
	{
		return position != other.position || velocity != other.velocity ||
			force != other.force || invMass != other.invMass || radius != other.radius ||
			correctionOn != other.correctionOn;
	}

	Vector2 PointMass::ComputeForce(const Vector2& Position, const Vector2& Velocity) const noexcept
	{
		Vector2 accel(0, 0);
		for (size_t i = 0; i < springs.size(); i++)
			accel += springInfo[i] ? springs[i].GetAForceVector(Position, Velocity) : springs[i].GetBForceVector(Position, Velocity);
		if (correctionOn)
			for (size_t i = 0; i < correctionSprings.size(); i++)
				accel += 0.5 * (correctionSpringInfo[i] ? correctionSprings[i].GetAForceVector(Position, Velocity) : correctionSprings[i].GetBForceVector(Position, Velocity));
		return (accel + force) * invMass;
	}
		
	void PointMass::AddSpring(const PointMassSpring& spring, bool isA) noexcept
	{
		springs.push_back(spring);
		springInfo.push_back(isA);
	}

	void PointMass::AddCorrectionSpring(const PointMassSpring& spring, bool isA) noexcept
	{
		correctionSprings.push_back(spring);
		correctionSpringInfo.push_back(isA);
	}

	f64 PointMass::KineticEnergy() const noexcept
	{
		if (!invMass)
			return 0;
		return 0.5 * (1 / invMass) * velocity.GetMagnitudeSquared();
	}

	bool PointMass::RemoveSpring(const PointMassSpring& spring) noexcept
	{
		for (size_t i = 0; i < springs.size(); i++)
		{
			if (springs[i] == spring)
			{
				springs.erase(springs.begin() + i);
				springInfo.erase(springInfo.begin() + i);
				return true;
			}
		}
		return false;
	}

	bool PointMass::RemoveCorrectionSpring(const PointMassSpring& spring) noexcept
	{
		for (size_t i = 0; i < correctionSprings.size(); i++)
		{
			if (correctionSprings[i] == spring)
			{
				correctionSprings.erase(correctionSprings.begin() + i);
				correctionSpringInfo.erase(correctionSpringInfo.begin() + i);
				return true;
			}
		}
		return false;
	}
}