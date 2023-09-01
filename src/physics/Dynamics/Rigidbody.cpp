#include "physics/Dynamics/Rigidbody.hpp"
#include <iostream>

namespace physics
{
	Rigidbody::Rigidbody() noexcept
	: Dynamicbody()
	{
	}

	Rigidbody::Rigidbody(const Collider& c, const Transform& t, const bool& isTrigger, const PhysicsMaterial& p,
		const f64& mass, bool usesGravity, const geo::Vector2& drag) noexcept
	: Dynamicbody(c, t, isTrigger, p, mass, usesGravity, drag)
	{
	}

	Rigidbody::Rigidbody(const Rigidbody& r) noexcept
	: Dynamicbody(r), isKinematic(r.isKinematic)
	{
	}

	Rigidbody::Rigidbody(Rigidbody && r) noexcept
	: Dynamicbody(r), isKinematic(r.isKinematic)
	{
	}

	Rigidbody::~Rigidbody() noexcept
	{
	}

	Rigidbody& Rigidbody::operator=(const Rigidbody& other) noexcept
	{
		Dynamicbody::operator=((const Dynamicbody&)other);
		isKinematic = other.isKinematic;
		return *this;
	}

	bool Rigidbody::operator==(const CollisionObject& other) const noexcept
	{
		if (typeid(other).name() != typeid(*this).name())
			return false;
		return Dynamicbody::operator==((const Dynamicbody&)other) && isKinematic == dynamic_cast<const Rigidbody&>(other).isKinematic;
	}

	bool Rigidbody::operator!=(const CollisionObject& other) const noexcept
	{
		if (typeid(other).name() != typeid(*this).name())
			return true;
		return Dynamicbody::operator!=((const Dynamicbody&)other) || isKinematic != dynamic_cast<const Rigidbody&>(other).isKinematic;
	}

	void Rigidbody::ApplyAngularForce(f64 force) noexcept
	{
		angularForce += force;
	}

	void Rigidbody::ApplyAngularImpulse(f64 force) noexcept
	{
		angularVelocity += force;
	}

	void Rigidbody::ApplyForce(const geo::Vector2& Force, const geo::Vector2& contactPoint) noexcept
	{
		if (!isStatic && !isKinematic)
		{
		 	force += Force;
			if (contactPoint != geo::Vector2::Infinity && Force.GetMagnitudeQuick())
			{
				torque = (collider->GetCenter() + transform.GetCOM()).Cross(Force);
				angularForce += torque;
			}
		}
	}

	void Rigidbody::ApplyImpulse(const geo::Vector2& impulse, const geo::Vector2& contactVec) noexcept
	{
		if (!isStatic && !isKinematic)
		{
			velocity += _invMass * impulse;
			if (contactVec != geo::Vector2::Infinity && impulse.GetMagnitudeQuick())
				angularVelocity += _invInertia * contactVec.Cross(impulse);
		}
	}

	CollisionObject* Rigidbody::Clone() const noexcept
	{
		return (CollisionObject*)new Rigidbody(*this);
	}

	void Rigidbody::Move(f64 offsetX, f64 offsetY) noexcept
	{
		if (isKinematic)
			transform.Translate(geo::Vector2(offsetX, offsetY));
	}

	void Rigidbody::Update(f64 dt) noexcept
	{
		if (!isKinematic && !isStatic)
		{	
			geo::Vector2 position = transform.GetPosition();
			Transform::RK4Integrate(&position.x, &velocity.x, &force.x, dt);
			Transform::RK4Integrate(&position.y, &velocity.y, &force.y, dt);
			f64 ang = transform.GetAngle();
			Transform::RK4Integrate(&ang, &angularVelocity, &angularForce, dt);
			transform.SetAngle(ang);
			transform.SetPosition(position);
			angularForce = 0;
			force.Set(0, 0);
		}
	}

}