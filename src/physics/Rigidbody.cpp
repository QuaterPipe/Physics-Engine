#include "../include/physics/Rigidbody.hpp"

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
	{
		this->staticFriction = staticFriction;
		this->kineticFriction = kineticFriction;
		this->restitution = restitution;
	}


	Rigidbody::Rigidbody() noexcept: CollisionObject()
	{
	}

	Rigidbody::Rigidbody(const Transform& t, Collider& c, bool isTrigger, f64 mass,
		bool usesGravity, f64 staticFriction, f64 kineticFriction,
		f64 restitution) noexcept : CollisionObject(t, c, isTrigger)
	{
		_isDynamic = true;
		_mass = mass;
		_usesGravity = usesGravity;
		_physicsMaterial.staticFriction = staticFriction;
		_physicsMaterial.kineticFriction = kineticFriction;
		_physicsMaterial.restitution = restitution;
	}

	Rigidbody::Rigidbody(const Rigidbody& r) noexcept
		:CollisionObject((const CollisionObject&)r)
	{
		_isDynamic = true;
		_mass = r.GetMass();
		if (_mass)
			_invMass = 1 / _mass;
		else
			_invMass = 0;
		
		_inertia = r.GetInertia();
		if (!_inertia)
			_invInertia = 1 / _inertia;
		else
			_invInertia = 0;
		_torque = r.GetTorque();
		_angularVelocity = r.GetAngularVelocity();
		_usesGravity = r.UsesGravity();
		_isKinematic = r.IsKinematic();
		_gravity = r.GetGravity();
		_velocity = r.GetVelocity();
		_drag = r.GetDrag();
		_physicsMaterial.staticFriction = r.GetStaticFriction();
		_physicsMaterial.kineticFriction = r.GetKineticFriction();
		_physicsMaterial.restitution = r.GetRestitution();
	}

	Rigidbody::~Rigidbody() noexcept
	{
	}

	Rigidbody& Rigidbody::operator=(const Rigidbody& other) noexcept
	{
		_mass = other.GetMass();
		_usesGravity = other.UsesGravity();
		_physicsMaterial.staticFriction = other.GetStaticFriction();
		_physicsMaterial.kineticFriction = other.GetKineticFriction();
		_physicsMaterial.restitution = other.GetRestitution();
		return *this;
	}

	void Rigidbody::ApplyAngularForce(f64 angularVelocity) noexcept
	{
		_angularVelocity += angularVelocity;
	}

	void Rigidbody::ApplyForce(const geometry::Vector& force, const geometry::Vector& contactPoint) noexcept
	{
	 	_velocity += force;
		if (contactPoint != geometry::Vector::Infinity && force.GetMagnitudeQuick())
		{
			geometry::Vector rF = _collider->GetCenterOfMass() - contactPoint;
			// τ = r*F*sinθ
			_torque += rF.GetMagnitudeQuick() * force.GetMagnitudeQuick() * sin(rF.Angle(force));
			_angularVelocity = _torque * _invInertia;
		}
	}

	void Rigidbody::ApplyImpulse(const geometry::Vector& impulse, const geometry::Vector& contactVec) noexcept
	{
		_velocity += _invMass * impulse;
	}

	CollisionObject* Rigidbody::Clone() const noexcept
	{
		return new Rigidbody(*this);
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
		return (_mass == r.GetMass()) && (_usesGravity == r.UsesGravity()) &&
			(_physicsMaterial.staticFriction == r.GetStaticFriction()) &&
			(_physicsMaterial.kineticFriction == r.GetKineticFriction()) &&
			(_physicsMaterial.restitution == r.GetRestitution()) && (_gravity == r.GetGravity()) &&
			(_velocity == r.GetVelocity()) && (_drag == r.GetDrag()) &&
			CollisionObject::Equals((const CollisionObject&) r);
	}

	f64 Rigidbody::GetAngularVelocity() const noexcept
	{
		return _angularVelocity;
	}

	geometry::Vector Rigidbody::GetDrag() const noexcept
	{
		return _drag;
	}

	geometry::Vector Rigidbody::GetForce() const noexcept
	{
		return _mass * _velocity;
	}

	geometry::Vector Rigidbody::GetGravity() const noexcept
	{
		return _gravity;
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

	f64 Rigidbody::GetKineticFriction() const noexcept
	{
		return _physicsMaterial.kineticFriction;
	}

	f64 Rigidbody::GetMass() const noexcept
	{
		return _mass;
	}

	PhysicsMaterial Rigidbody::GetPhysicsMaterial() const noexcept
	{
		return _physicsMaterial;
	}

	f64 Rigidbody::GetRestitution() const noexcept
	{
		return _physicsMaterial.restitution;
	}

	f64 Rigidbody::GetStaticFriction() const noexcept
	{
		return _physicsMaterial.staticFriction;
	}

	f64 Rigidbody::GetTorque() const noexcept
	{
		return _torque;
	}

	geometry::Vector Rigidbody::GetVelocity() const noexcept
	{
		return _velocity;
	}

	bool Rigidbody::IsKinematic() const noexcept
	{
		return _isKinematic;
	}

	void Rigidbody::Move(f64 offsetX, f64 offsetY) noexcept
	{
		if (_isKinematic)
		{
			_transform.position += geometry::Vector(offsetX, offsetY);
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
		return (_mass != r.GetMass()) || (_usesGravity != r.UsesGravity()) ||
			(_physicsMaterial.staticFriction != r.GetStaticFriction()) ||
			(_physicsMaterial.kineticFriction != r.GetKineticFriction()) ||
			(_physicsMaterial.restitution != r.GetRestitution()) || (_gravity != r.GetGravity()) ||
			(_velocity != r.GetVelocity()) || (_drag != r.GetDrag()) ||
			CollisionObject::NotEquals((const CollisionObject&) r);
	}

	void Rigidbody::SetAngularVelocity(f64 angularVelocity) noexcept
	{
		_angularVelocity = angularVelocity;
	}

	void Rigidbody::SetDrag(const geometry::Vector& drag) noexcept
	{
		_drag = drag;
	}

	void Rigidbody::SetGravity(const geometry::Vector& grav) noexcept
	{
		_gravity = grav;
	}

	void Rigidbody::SetIsKinematic(bool isKinematic) noexcept
	{
		_isKinematic = isKinematic;
	}

	void Rigidbody::SetInertia(f64 inertia) noexcept
	{
		_inertia = inertia;
		if (_inertia)
			_invInertia = 1 / _inertia;
		else
			_invInertia = 0;
	}

	void Rigidbody::SetKineticFriction(f64 kineticFriction) noexcept
	{
		_physicsMaterial.kineticFriction = kineticFriction;
	}

	void Rigidbody::SetMass(f64 mass) noexcept
	{
		_mass = mass;
		if (mass)
			_invMass = 1 / mass;
		else
			_invMass = 0;
	}

	void Rigidbody::SetPhysicsMaterial(const PhysicsMaterial& physicsMaterial) noexcept
	{
		_physicsMaterial = physicsMaterial;
	}

	void Rigidbody::SetRestitution(f64 restitution) noexcept
	{
		_physicsMaterial.restitution = restitution;
	}

	void Rigidbody::SetStaticFriction(f64 staticFriction) noexcept
	{
		_physicsMaterial.staticFriction = staticFriction;
	}

	void Rigidbody::SetTorque(f64 torque) noexcept
	{
		_torque = torque;
	}

	void Rigidbody::SetUsesGravity(bool usesGravity) noexcept
	{
		_usesGravity = usesGravity;
	}

	void Rigidbody::SetVelocity(const geometry::Vector& vel) noexcept
	{
		_velocity = vel;
	}

	bool Rigidbody::UsesGravity() const noexcept
	{
		return _usesGravity;
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