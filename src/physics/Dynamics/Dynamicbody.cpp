#include "physics/Dynamics/Dynamicbody.hpp"

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

	RK4State::RK4State()
	{
	}

	Dynamicbody::Dynamicbody() noexcept
	: CollisionObject()
	{
		_isDynamic = true;
	}

	Dynamicbody::Dynamicbody(const Collider& c, const Transform& t, bool isTrigger, const PhysicsMaterial& p,
		f64 mass, bool usesGravity, const geo::Vector2& drag) noexcept
	: CollisionObject(c, t, isTrigger),  _mass(mass), drag(drag), physicsMaterial(p), usesGravity(usesGravity)
	{
		_isDynamic = true;
		_mass = mass;
		_invMass = _mass ? 1 / _mass : 0;

	}

	Dynamicbody::Dynamicbody(const Dynamicbody& d) noexcept
	: CollisionObject(d), _mass(d.GetMass()), _inertia(d.GetInertia()), gravity(d.gravity), velocity(d.velocity), drag(d.drag),
		force(d.force), angularVelocity(d.angularVelocity), angularForce(d.angularForce), physicsMaterial(d.physicsMaterial),
		usesGravity(d.usesGravity), isStatic(d.isStatic), appliedForce(d.appliedForce), appliedAngularForce(d.appliedAngularForce)
	{
		_isDynamic = true;
		_invMass = _mass ? 1 / _mass : 0;
	}

	Dynamicbody::Dynamicbody(Dynamicbody&& d) noexcept
		: CollisionObject((CollisionObject&&)d), _mass(d.GetMass()), _inertia(d.GetInertia()),
		gravity(d.gravity), velocity(d.velocity), drag(d.drag), force(d.force), angularVelocity(d.angularVelocity),
		angularForce(d.angularForce), physicsMaterial(d.physicsMaterial), usesGravity(d.usesGravity), isStatic(d.isStatic),
		appliedForce(d.appliedForce), appliedAngularForce(d.appliedAngularForce)
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
		appliedForce = d.appliedForce;
		appliedAngularForce = d.appliedAngularForce;
		return *this;
	}

	bool Dynamicbody::operator==(const CollisionObject& other) const noexcept
	{
		if (typeid(other).name() != (typeid(*this).name()))
			return false;
		const Dynamicbody& o = dynamic_cast<const Dynamicbody&>(other);
		return CollisionObject::operator==(other) && _mass == o.GetMass() &&
			_inertia == o.GetInertia() && physicsMaterial == o.physicsMaterial &&
			usesGravity == o.usesGravity && gravity == o.gravity && velocity == o.velocity &&
			drag == o.drag && angularVelocity == o.angularVelocity && angularForce == o.angularForce &&
			isStatic == o.isStatic && force == o.force && appliedForce == o.appliedForce &&
			appliedAngularForce == o.appliedAngularForce;
	}

	bool Dynamicbody::operator!=(const CollisionObject& other) const noexcept
	{
		if (typeid(other).name() != typeid(*this).name())
			return true;
		const Dynamicbody& o = dynamic_cast<const Dynamicbody&>(other);
		return CollisionObject::operator!=(other) || _mass != o.GetMass() ||
			_inertia != o.GetInertia() || physicsMaterial != o.physicsMaterial ||
			usesGravity != o.usesGravity || gravity != o.gravity || velocity != o.velocity ||
			drag != o.drag || angularVelocity != o.angularVelocity || angularForce != o.angularForce ||
			isStatic != o.isStatic || force != o.force || appliedForce != o.appliedForce ||
			appliedAngularForce != o.appliedAngularForce;
	}

	void Dynamicbody::AddConstraint(Constraint* constraint) noexcept
	{
		constraints.push_back(constraint);
	}

	f64 Dynamicbody::GetInertia() const noexcept
	{
		return _inertia;
	}

	f64 Dynamicbody::GetInvInertia() const noexcept
	{
		if (isStatic)
			return 0;
		return _invInertia;
	}
 
	f64 Dynamicbody::GetInvMass() const noexcept
	{
		if (isStatic)
			return 0;
		return _invMass;
	}

	f64 Dynamicbody::GetMass() const noexcept
	{
		return _mass;
	}

	void Dynamicbody::RemoveConstraint(Constraint* constraint) noexcept
	{
		for (auto p = constraints.begin(); p < constraints.end(); p++)
		{
			if (*p == constraint)
			{
				constraints.erase(p);
				return;
			}
		}
	}

	void Dynamicbody::SetInertia(f64 inertia) noexcept
	{
		_inertia = inertia;
		_invInertia = inertia ? 1 / inertia : 0;
	}

	void Dynamicbody::SetMass(f64 mass) noexcept
	{
		_mass = mass;
		_invMass = mass ? 1 / mass : 0;
	}
}