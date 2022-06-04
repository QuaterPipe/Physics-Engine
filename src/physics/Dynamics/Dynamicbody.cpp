#include "../../include/physics/Dynamics/Dynamicbody.hpp"

namespace physics
{
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

	Dynamicbody::Dynamicbody() noexcept
	: CollisionObject()
	{
		_isDynamic = true;
	}

	Dynamicbody::Dynamicbody(const Collider& c, const Transform& t, const bool& isTrigger, const PhysicsMaterial& p,
		const f64& mass, bool usesGravity, const geo::Vector& drag) noexcept
	: CollisionObject(c, t, isTrigger),  _mass(mass), drag(drag), physicsMaterial(p), usesGravity(usesGravity)
	{
		_isDynamic = true;
		_mass = mass;
		_invMass = _mass ? 1 / _mass : 0;
	}

	Dynamicbody::Dynamicbody(const Dynamicbody& d) noexcept
	: CollisionObject((const CollisionObject&)d), _mass(d.GetMass()), _inertia(d.GetInertia()), gravity(d.gravity), velocity(d.velocity), drag(d.drag),
		force(d.force), angularVelocity(d.angularVelocity), angularForce(d.angularForce), physicsMaterial(d.physicsMaterial), usesGravity(d.usesGravity), isStatic(d.isStatic)
	{
		_isDynamic = true;
		_invMass = _mass ? 1 / _mass : 0;
	}

	Dynamicbody::Dynamicbody(Dynamicbody && d) noexcept
	: CollisionObject((CollisionObject &&)d), _mass(d.GetMass()), _inertia(d.GetInertia()),
		gravity(d.gravity), velocity(d.velocity), drag(d.drag), force(d.force), angularVelocity(d.angularVelocity),
		angularForce(d.angularForce), physicsMaterial(d.physicsMaterial), usesGravity(d.usesGravity), isStatic(d.isStatic)
	{
		_isDynamic = true;
		_invMass = _mass ? 1 / _mass : 0;
		_invInertia = _inertia ? 1 / _inertia : 0;
	}

	Dynamicbody& Dynamicbody::operator=(const Dynamicbody& d) noexcept
	{
		CollisionObject::operator=((CollisionObject&)d);
		SetMass(d.GetMass());
		SetInertia(d.GetInertia());
		physicsMaterial = d.physicsMaterial;
		usesGravity = d.usesGravity;
		gravity = d.gravity;
		velocity = d.velocity;
		drag = d.drag;
		angularVelocity = d.angularVelocity;
		angularForce = d.angularForce;
		isStatic = d.isStatic;
		force = d.force;
		return *this;
	}

	bool Dynamicbody::Equals(const Dynamicbody& other) const noexcept
	{
		return CollisionObject::Equals(other) && _mass == other.GetMass() &&
			_inertia == other.GetInertia() && physicsMaterial == other.physicsMaterial &&
			usesGravity == other.usesGravity && gravity == other.gravity && velocity == other.velocity &&
			drag == other.drag && angularVelocity == other.angularVelocity && angularForce == other.angularForce &&
			isStatic == other.isStatic && force == other.force;
	}

	std::vector<unsigned char> Dynamicbody::GetBytes() const noexcept
	{
		return ToBytes(this, sizeof(*this));
	}

	f64 Dynamicbody::GetInertia() const noexcept
	{
		return _inertia;
	}

	f64 Dynamicbody::GetInvInertia() const noexcept
	{
		return _invInertia;
	}
 
	f64 Dynamicbody::GetInvMass() const noexcept
	{
		return _invMass;
	}

	f64 Dynamicbody::GetMass() const noexcept
	{
		return _mass;
	}

	bool Dynamicbody::NotEquals(const Dynamicbody& other) const noexcept
	{
		return CollisionObject::NotEquals((CollisionObject&)other) || _mass != other.GetMass() ||
			_inertia != other.GetInertia() || physicsMaterial != other.physicsMaterial ||
			usesGravity != other.usesGravity || gravity != other.gravity || velocity != other.velocity ||
			drag != other.drag || angularVelocity != other.angularVelocity || angularForce != other.angularForce ||
			isStatic != other.isStatic || force != other.force;
	}

	void Dynamicbody::SetInertia(const f64& inertia) noexcept
	{
		_inertia = inertia;
		_invInertia = inertia ? 1 / inertia : 0;
	}

	void Dynamicbody::SetMass(const f64& mass) noexcept
	{
		_mass = mass;
		_invMass = mass ? 1 / mass : 0;
	}

	void Dynamicbody::Update(f64 dt) noexcept
	{
		for (Joint* j: joints)
			j->Update(dt);
	}
}