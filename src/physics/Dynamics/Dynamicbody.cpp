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

	bool Dynamicbody::operator==(const CollisionObject& other) const noexcept
	{
		if (typeid(other).name() != (typeid(*this).name()))
			return false;
		const Dynamicbody& o = dynamic_cast<const Dynamicbody&>(other);
		return CollisionObject::operator==(other) && _mass == o.GetMass() &&
			_inertia == o.GetInertia() && physicsMaterial == o.physicsMaterial &&
			usesGravity == o.usesGravity && gravity == o.gravity && velocity == o.velocity &&
			drag == o.drag && angularVelocity == o.angularVelocity && angularForce == o.angularForce &&
			isStatic == o.isStatic && force == o.force;
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
			isStatic != o.isStatic || force != o.force;
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

	
	void Dynamicbody::IntegrateForces(f64 dt) noexcept
	{
		if (isStatic)
			return;
		velocity += (force * _invMass + gravity) * (dt / 2.0);
  		angularVelocity += angularForce * _invInertia * (dt / 2.0);
	}

	void Dynamicbody::IntegrateVelocity(f64 dt) noexcept
	{
		if (isStatic)
			return;
		position += velocity * dt;
		rotation = geo::Matrix2(transform.GetAngle() + angularVelocity * dt);
		IntegrateForces(dt);
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

	void Dynamicbody::Update(f64 dt) noexcept
	{
		for (Joint* j: joints)
			j->Update(dt);
	}
}