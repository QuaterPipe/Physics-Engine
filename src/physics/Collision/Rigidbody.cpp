#include "../../include/physics/Collision/Rigidbody.hpp"
#include "../../include/physics/Tools/OstreamOverloads.hpp"
#include <iostream>

namespace physics
{
	using namespace serialization;

	Rigidbody::Rigidbody() noexcept
	: Dynamicbody()
	{
	}

	Rigidbody::Rigidbody(const Collider& c, const Transform& t, const bool& isTrigger, const PhysicsMaterial& p,
		const f64& mass, bool usesGravity, const geometry::Vector& drag) noexcept
	: Dynamicbody(c, t, isTrigger, p, mass, usesGravity, drag)
	{
	}

	Rigidbody::Rigidbody(const Rigidbody& r) noexcept
	: Dynamicbody((const Dynamicbody&)r), isKinematic(r.isKinematic)
	{
	}

	Rigidbody::Rigidbody(Rigidbody && r) noexcept
	: Dynamicbody((const Dynamicbody &&)r), isKinematic(r.isKinematic)
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

	void Rigidbody::ApplyAngularForce(f64 angularVelocity) noexcept
	{
		angularVelocity += angularVelocity;
	}

	void Rigidbody::ApplyForce(const geometry::Vector& force, const geometry::Vector& contactPoint) noexcept
	{
	 	velocity += force;
		if (contactPoint != geometry::Vector::Infinity && force.GetMagnitudeQuick())
		{
			geometry::Vector rF = collider->GetCenter() - contactPoint;
			// τ = r*F*sin(θ)
			SetInertia(_mass);
			torque = rF.GetMagnitudeQuick() * force.GetMagnitudeQuick() * sin(rF.Angle(force));
			angularVelocity += torque * _invInertia;
		}
	}

	void Rigidbody::ApplyImpulse(const geometry::Vector& impulse, const geometry::Vector& contactVec) noexcept
	{
		velocity += _invMass * impulse;
	}

	CollisionObject* Rigidbody::Clone() const noexcept
	{
		return (CollisionObject*)new Rigidbody(*this);
	}

	bool Rigidbody::Equals(const Hashable& other) const noexcept
	{
		Rigidbody r;
		try
		{
			r = dynamic_cast<const Rigidbody&>(other);
		}
		catch(const std::bad_cast& e)
		{
			return false;
		}
		return (_mass == r.GetMass()) && (usesGravity == r.usesGravity) &&
			physicsMaterial == r.physicsMaterial && (gravity == r.gravity) &&
			(velocity == r.velocity) && (drag == r.drag) &&
			CollisionObject::Equals((const CollisionObject&) r);
	}

	void Rigidbody::Move(f64 offsetX, f64 offsetY) noexcept
	{
		if (isKinematic)
		{
			transform.position += geometry::Vector(offsetX, offsetY);
		}
	}

	bool Rigidbody::NotEquals(const Hashable& other) const noexcept
	{
		Rigidbody r;
		try
		{
			r = dynamic_cast<const Rigidbody&>(other);
		}
		catch(const std::bad_cast& e)
		{
			return true;
		}
		return (_mass != r.GetMass()) || (usesGravity != r.usesGravity) ||
			(physicsMaterial != r.physicsMaterial) || (gravity != r.gravity) ||
			(velocity != r.velocity) || (drag != r.drag) ||
			CollisionObject::NotEquals((const CollisionObject&) r);
	}

	void Rigidbody::Update(f64 dt) const noexcept
	{
	}

	Serializable* Rigidbody::Deserialize(const std::vector<byte>& v,
			const size_t& index, const size_t& length) const
	{
		return NULL;
	}

	unsigned char Rigidbody::GetByte(const size_t& index) const
	{
		return 0x01;
	}

	unsigned long Rigidbody::TotalByteSize() const noexcept
	{
		return 0UL;
	}

	std::vector<unsigned char> Rigidbody::Serialize() const noexcept
	{
		std::vector<unsigned char> vec;
		return vec;
	}
}