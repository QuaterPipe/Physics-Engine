#include "../../include/physics/Dynamics/Rigidbody.hpp"
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
		const f64& mass, bool usesGravity, const geo::Vector& drag) noexcept
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

	void Rigidbody::ApplyAngularForce(f64 force) noexcept
	{
		angularVelocity += force;
	}

	void Rigidbody::ApplyForce(const geo::Vector& Force, const geo::Vector& contactPoint) noexcept
	{
		if (!isStatic && !isKinematic)
		{
		 	this->force += Force;
			if (contactPoint != geo::Vector::Infinity && Force.GetMagnitudeQuick())
			{
				torque = (collider->GetCenter() + centerOfRotation).Cross(Force);
				angularForce += torque;
			}
		}
	}

	void Rigidbody::ApplyImpulse(const geo::Vector& impulse, const geo::Vector& contactVec) noexcept
	{
		if (!isStatic && !isKinematic)
		{
			velocity += _invMass * impulse;
			if (contactVec != geo::Vector::Infinity && impulse.GetMagnitudeQuick())
				angularVelocity += _invInertia * contactVec.Cross(impulse);
		}
	}

	CollisionObject* Rigidbody::Clone() const noexcept
	{
		return (CollisionObject*)new Rigidbody(*this);
	}

	bool Rigidbody::Equals(const Rigidbody& other) const noexcept
	{
		return Dynamicbody::Equals(other) && isKinematic == other.isKinematic;
	}

	std::vector<unsigned char> Rigidbody::GetBytes() const noexcept
	{
		return ToBytes(this, sizeof(*this));
	}

	void Rigidbody::Move(f64 offsetX, f64 offsetY) noexcept
	{
		if (isKinematic)
			transform.position += geo::Vector(offsetX, offsetY);
	}

	bool Rigidbody::NotEquals(const Rigidbody& other) const noexcept
	{
		return Dynamicbody::NotEquals(other) || isKinematic != other.isKinematic;
	}

	void Rigidbody::Update(f64 dt) noexcept
	{
		if (!isKinematic && !isStatic)
		{
			velocity += (force * _invMass) * dt;
			position += velocity * dt;
			if (angularForce)
				angularVelocity += (angularForce * _invInertia) * dt;
			rotation = rotation * geo::Matrix2(angularVelocity * dt);
			angularForce = 0;
			force.Set(0, 0);
		}
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