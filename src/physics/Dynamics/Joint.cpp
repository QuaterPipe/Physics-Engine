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
		if (DistanceSquared(a->transform.GetPosition() + a->transform.GetCOM(), b->transform.GetPosition() + b->transform.GetCOM()) < SQRD(length))
		{
	 		f64 dis = Distance(a->transform.GetPosition() + a->transform.GetCOM(), b->transform.GetPosition() + b->transform.GetCOM()) - length;
			a->transform.Translate(dis * (a->GetMass() / (a->GetMass() + b->GetMass())) * ((b->transform.GetPosition() + b->transform.GetCOM()) - (a->transform.GetPosition() + a->transform.GetCOM())).Normalized());
			b->transform.Translate(dis * (b->GetMass() / (a->GetMass() + b->GetMass())) * ((a->transform.GetPosition() + a->transform.GetCOM()) - (b->transform.GetPosition() + b->transform.GetCOM())).Normalized());
		}
		a->ApplyForce(b->GetMass() * b->velocity * dt);
		b->ApplyForce(a->GetMass() * a->velocity * dt);
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
		if (a->transform.GetCOM() != aContact)
			a->transform.SetCOM(aContact);
		if (b->transform.GetCOM() != aContact)
			b->transform.SetCOM(aContact);
		a->angularVelocity *= (1 - angularFriction) * dt;
		b->angularVelocity *= (1 - angularFriction) * dt;
	}
}