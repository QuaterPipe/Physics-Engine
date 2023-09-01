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
		if (geo::DistanceSquared(a->transform.GetPosition() + a->transform.GetCOM(), b->transform.GetPosition() + b->transform.GetCOM()) < SQRD(length))
		{
	 		f64 dis = geo::Distance(a->transform.GetPosition() + a->transform.GetCOM(), b->transform.GetPosition() + b->transform.GetCOM()) - length;
			a->transform.Translate(dis * (a->GetMass() / (a->GetMass() + b->GetMass())) * ((b->transform.GetPosition() + b->transform.GetCOM()) - (a->transform.GetPosition() + a->transform.GetCOM())).Normalized());
			b->transform.Translate(dis * (b->GetMass() / (a->GetMass() + b->GetMass())) * ((a->transform.GetPosition() + a->transform.GetCOM()) - (b->transform.GetPosition() + b->transform.GetCOM())).Normalized());
		}
		a->ApplyForce(b->GetMass() * b->velocity * dt);
		b->ApplyForce(a->GetMass() * a->velocity * dt);
	}

	SpringJoint::SpringJoint(Dynamicbody* a, Dynamicbody* b, f64 length, f64 stiffness, f64 dampingFactor) noexcept
	: DistanceJoint(a, b, length), stiffness(stiffness), dampingFactor(dampingFactor)
	{
	}

	f64 SpringJoint::ForceExerting() const noexcept
	{
		f64 Fs = (geo::Distance(a->transform.GetPosition() + a->transform.GetCOM(), b->transform.GetPosition() + b->transform.GetCOM()) - length) * stiffness;
		f64 Fd = ((b->transform.GetPosition() + b->transform.GetCOM()) - (a->transform.GetPosition() + a->transform.GetCOM())).Normalized().Dot(b->velocity - a->velocity) * dampingFactor;
		return Fs + Fd;
	}

	void SpringJoint::Update(f64 dt) noexcept
	{
		a->ApplyForce(ForceExerting() * ((b->transform.GetPosition() + b->transform.GetCOM()) - (a->transform.GetPosition() + a->transform.GetCOM())).Normalized() * dt);
		b->ApplyForce(ForceExerting() * ((a->transform.GetPosition() + a->transform.GetCOM()) - (b->transform.GetPosition() + b->transform.GetCOM())).Normalized() * dt);
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