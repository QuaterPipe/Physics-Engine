#include "physics/Dynamics/Dynamicbody.hpp"

namespace physics
{
	DistanceJoint::DistanceJoint(Dynamicbody* a, Dynamicbody* b, f64 length) noexcept
	: length(length)
	{
		this->a = a;
		this->b = b;
	}

	void DistanceJoint::Update(f64 dt) noexcept
	{
		if (geo::DistanceSquared(a->position, b->position) < SQRD(length))
		{
	 		f64 dis = geo::Distance(a->position, b->position) - length;
			a->position += dis * (a->GetMass() / (a->GetMass() + b->GetMass())) * (b->position - a->position).Normalized();
			b->position += dis * (b->GetMass() / (a->GetMass() + b->GetMass())) * (a->position - b->position).Normalized();
		}
		a->ApplyForce(dt, b->GetMass() * b->velocity);
		b->ApplyForce(dt, a->GetMass() * a->velocity);
	}

	SpringJoint::SpringJoint(Dynamicbody* a, Dynamicbody* b, f64 length, f64 stiffness, f64 dampingFactor) noexcept
	: DistanceJoint(a, b, length), stiffness(stiffness), dampingFactor(dampingFactor)
	{
	}

	f64 SpringJoint::ForceExerting() const noexcept
	{
		f64 Fs = (geo::Distance(a->position, b->position) - length) * stiffness;
		f64 Fd = (b->position - a->position).Normalized().Dot(b->velocity - a->velocity) * dampingFactor;
		return Fs + Fd;
	}

	void SpringJoint::Update(f64 dt) noexcept
	{
		a->ApplyForce(dt, ForceExerting() * (b->position - a->position).Normalized());
		b->ApplyForce(dt, ForceExerting() * (a->position - b->position).Normalized());
	}

	HingeJoint::HingeJoint(Dynamicbody* a, Dynamicbody* b, f64 angularFriction) noexcept
	: angularFriction(angularFriction)
	{
		this->a = a;
		this->b = b;
	}

	void HingeJoint::Update(f64 dt) noexcept
	{
		aContact = bContact;
		if (a->centerOfRotation != aContact)
			a->centerOfRotation = aContact;
		if (b->centerOfRotation != aContact)
			b->centerOfRotation = aContact;
		a->angularVelocity *= (1 - angularFriction) * dt;
		b->angularVelocity *= (1 - angularFriction) * dt;
	}
}