#include "../../include/physics/Collision/Rigidbody.hpp"
#include "../../include/physics/Tools/OstreamOverloads.hpp"
#include <iostream>

namespace physics
{
	using namespace serialization;

	const PhysicsMaterial PhysicsMaterial::Steel = PhysicsMaterial(0.74, 0.57, 0.63);
	const PhysicsMaterial PhysicsMaterial::Glass = PhysicsMaterial(0.94, 0.40, 0.69);
	const PhysicsMaterial PhysicsMaterial::Ice = PhysicsMaterial(0.10, 0.03, 0.63);

	PhysicsMaterial::PhysicsMaterial() noexcept
	{
	}

	PhysicsMaterial::PhysicsMaterial(f64 staticFriction, f64 kineticFriction, f64 restitution) noexcept
	: staticFriction(staticFriction), kineticFriction(kineticFriction), restitution(restitution)
	{
	}

	bool PhysicsMaterial::operator==(const PhysicsMaterial& other) const noexcept
	{
		return staticFriction == other.staticFriction && kineticFriction == other.kineticFriction &&
			restitution == other.restitution;
	}

	bool PhysicsMaterial::operator!=(const PhysicsMaterial& other) const noexcept
	{
		return staticFriction != other.staticFriction || kineticFriction != other.kineticFriction ||
			restitution != other.restitution;
	}

	Rigidbody::Rigidbody() noexcept
	: CollisionObject()
	{
	}

	Rigidbody::Rigidbody(const Transform& t, Collider& c, bool isTrigger, f64 mass,
		bool usesGravity, f64 staticFriction, f64 kineticFriction,
		f64 restitution) noexcept
	: CollisionObject(t, c, isTrigger), _mass(mass), usesGravity(usesGravity),
	physicsMaterial(staticFriction, kineticFriction, restitution)
	{
		_isDynamic = true;
	}

	Rigidbody::Rigidbody(const Rigidbody& r) noexcept
	: CollisionObject((const CollisionObject&)r), _mass(r.GetMass()), torque(r.torque), 
	angularVelocity(r.angularVelocity), isKinematic(r.isKinematic), gravity(r.gravity),
	velocity(r.velocity), drag(r.drag), physicsMaterial(r.staticFriction, r.kineticFriction, r.restitution)
	{
		_isDynamic = true;
		_mass = r._mass;
		if (_mass)
			_invMass = 1 / _mass;
		else
			_invMass = 0;
		_inertia = r.GetInertia();
		if (!_inertia)
			_invInertia = 1 / _inertia;
		else
			_invInertia = 0;
	}

	Rigidbody::Rigidbody(Rigidbody && r) noexcept
	: CollisionObject((const CollisionObject &&)r), _mass(r.GetMass()), torque(r.torque), 
	angularVelocity(r.angularVelocity), isKinematic(r.isKinematic), gravity(r.gravity),
	velocity(r.velocity), drag(r.drag), physicsMaterial(r.staticFriction, r.kineticFriction, r.restitution)
	{
		_isDynamic = true;
		_mass = r._mass;
		if (_mass)
			_invMass = 1 / _mass;
		else
			_invMass = 0;
		_inertia = r.GetInertia();
		if (!_inertia)
			_invInertia = 1 / _inertia;
		else
			_invInertia = 0;
	}

	Rigidbody::~Rigidbody() noexcept
	{
	}

	Rigidbody& Rigidbody::operator=(const Rigidbody& other) noexcept
	{
		CollisionObject::operator=((const CollisionObject&)other);
		_mass = other.GetMass();
		isKinematic = other.isKinematic;
		usesGravity = other.GetMass();
		physicsMaterial = other.physicsMaterial;
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
			// τ = r*F*sinθ
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

	f64 Rigidbody::GetInertia() const noexcept
	{
		return _inertia;
	}

	f64 Rigidbody::GetInvInertia() const noexcept
	{
		return _invInertia;
	}

	f64 Rigidbody::GetInvMass() const noexcept
	{
		return _invMass;
	}

	f64 Rigidbody::GetMass() const noexcept
	{
		return _mass;
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

	void Rigidbody::SetMass(const f64& mass) noexcept
	{
		_mass = mass;
		if (_mass)
			_invMass = 1 / _mass;
		else
			_invMass = 0;
	}

	void Rigidbody::SetInertia(const f64& inertia) noexcept
	{
		_inertia = inertia;
		if (_inertia)
			_invInertia = 1 / _inertia;
		else
			_invInertia = 0;
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