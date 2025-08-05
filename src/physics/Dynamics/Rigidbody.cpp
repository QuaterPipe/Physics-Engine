#include "physics/Dynamics/Rigidbody.hpp"
#include "physics/Dynamics/Integration.hpp"


namespace physics
{
	Rigidbody::Rigidbody() noexcept
	: Dynamicbody()
	{
	}

	Rigidbody::Rigidbody(const Collider& c, const Transform& t, const bool& isTrigger, const PhysicsMaterial& p,
		const f64& mass, bool usesGravity, f64 dragCoef) noexcept
	: Dynamicbody(c, t, isTrigger, p, mass, usesGravity, dragCoef)
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

	Rigidbody& Rigidbody::operator=(Rigidbody&& other) noexcept
	{
		Dynamicbody::operator=((Dynamicbody&&)other);
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

	void Rigidbody::ApplyForce(const Vector2& Force, const Vector2& contactPoint) noexcept
	{
		if (!isStatic && !isKinematic)
		{
		 	_force += Force;
			if (contactPoint != Vector2::Infinity && Force.GetMagnitudeExact())
			{
				angularForce += (transform.GetCOM()).Cross(Force);
			}
		}
	}

	void Rigidbody::ApplyImpulse(const Vector2& impulse, const Vector2& contactVec) noexcept
	{
		if (!isStatic && !isKinematic)
		{
			velocity += _invMass * impulse;
			if (contactVec != Vector2::Infinity && impulse.GetMagnitudeExact())
				angularVelocity += _invInertia * contactVec.Cross(impulse);
		}
	}

	f64 Rigidbody::ComputeAngularForce(f64 orientation, f64 angVelocity) const noexcept
	{
		return appliedAngularForce;
	}

	Vector2 Rigidbody::ComputeForce(const Vector2& position, const Vector2 &velocity, f64 orient) const noexcept
	{
		Vector2 normV = velocity.Normalized();
		Vector2 dragForce = -normV;
		normV.Rotate(Vector2::Origin, orient);
		dragForce *= 0.5 * fluidDensity * velocity.GetMagnitudeSquared() * dragCoefficient * collider->CrossSectionalArea(normV);
		return appliedForce + dragForce;
	}

	CollisionObject* Rigidbody::Clone() const noexcept
	{
		return (CollisionObject*)new Rigidbody(*this);
	}

	void Rigidbody::Move(f64 offsetX, f64 offsetY) noexcept
	{
		if (isKinematic)
			transform.Translate(Vector2(offsetX, offsetY));
	}

	void Rigidbody::Update(f64 dt, int RK4Step) noexcept
	{
		if (!isKinematic && !isStatic)
		{
			Vector2 position = transform.GetPosition();
			f64 orient = transform.GetAngle();
			switch (RK4Step)
			{
				case 0:
					posState.a1 = ComputeForce(position, velocity, orient);
					posState.k1X = velocity;
					posState.k1V = posState.a1;

					angleState.a1.x = ComputeAngularForce(orient, angularVelocity);
					angleState.k1X.x = angularVelocity;
					angleState.k1V = angleState.a1;
					break;
				case 1:
					posState.tmpX = position + 0.5 * dt * posState.k1X;
					posState.tmpV = velocity + 0.5 * dt * posState.k1V;
					posState.a2 = ComputeForce(posState.tmpX, posState.tmpV, angleState.k1X.x);
					posState.k2X = posState.tmpV;
					posState.k2V = posState.a2;

					angleState.tmpX = orient + 0.5 * dt * angleState.k1X;
					angleState.tmpV = angularVelocity + 0.5 * dt * angleState.k1V;
					angleState.a2.x = ComputeAngularForce(angleState.tmpX.x, angleState.tmpV.x);
					angleState.k2X = angleState.tmpV;
					angleState.k2V = angleState.a2;
					break;
				case 2:
					posState.tmpX = position + 0.5 * dt * posState.k2X;
					posState.tmpV = velocity + 0.5 * dt * posState.k2V;
					posState.a3 = ComputeForce(posState.tmpX, posState.tmpV, angleState.k2X.x);
					posState.k3X = posState.tmpV;
					posState.k3V = posState.a3;

					angleState.tmpX = orient + 0.5 * dt * angleState.k2X;
					angleState.tmpV = angularVelocity + 0.5 * dt * angleState.k2V;
					angleState.a3.x = ComputeAngularForce(angleState.tmpX.x, angleState.tmpV.x);
					angleState.k3X = angleState.tmpV;
					angleState.k3V = angleState.a3;
					break;
				case 3:
					posState.tmpX = position + dt * posState.k3X;
					posState.tmpV = velocity + dt * posState.k3V;
					posState.a4 = ComputeForce(posState.tmpX, posState.tmpV, angleState.k3X.x);
					posState.k4X = posState.tmpV;
					posState.k4V = posState.a4;

					angleState.tmpX = orient + dt * angleState.k3X;
					angleState.tmpV = angularVelocity + dt * angleState.k3V;
					angleState.a4.x = ComputeAngularForce(angleState.tmpX.x, angleState.tmpV.x);
					angleState.k4X = angleState.tmpV;
					angleState.k4V = angleState.a4;

					position = position + (dt / 6.0) * (posState.k1X + 2 * posState.k2X + 2 * posState.k3X + posState.k4X);
					velocity = velocity + (dt / 6.0) * (posState.k1V + 2 * posState.k2V + 2 * posState.k3V + posState.k4V);

					orient = orient + (dt / 6.0) * (angleState.k1X.x + 2 * angleState.k2X.x + 2 * angleState.k3X.x + angleState.k4X.x);
					angularVelocity = angularVelocity + (dt / 6.0) * (angleState.k1V.x + 2 * angleState.k2V.x + 2 * angleState.k3V.x + angleState.k4V.x);
					
					transform.SetPosition(position);
					transform.SetAngle(orient);
					appliedAngularForce = 0;
					appliedForce.Set(0, 0);
					_force.Set(0, 0);
					angularForce = 0;
					break;
				default:
					break;
			}
		}
	}

	Rigidbody Rigidbody::CreateBox(f64 width, f64 height, f64 x, f64 y, f64 mass, bool isStatic) noexcept
	{
		Transform trans;
		trans.SetPosition(x, y);
		BoxCollider box(width, height);
		Rigidbody r(box, trans, false, PhysicsMaterial(), mass);
		r.isStatic = isStatic;
		return r;
	}

	Rigidbody Rigidbody::CreateCircle(f64 radius, f64 x, f64 y, f64 mass, bool isStatic) noexcept
	{
		Transform trans;
		trans.SetPosition(x, y);
		CircleCollider circ(radius);
		Rigidbody r(circ, trans, false, PhysicsMaterial(), mass);
		r.isStatic = isStatic;
		return r;
	}

	Rigidbody Rigidbody::CreatePolygon(size_t nSides, f64 edgeLength, f64 x, f64 y, f64 mass, bool isStatic) noexcept
	{
		Transform trans;
		trans.SetPosition(x, y);
		PolygonCollider poly(edgeLength, nSides);
		Rigidbody r(poly, trans, false, PhysicsMaterial(), mass);
		r.isStatic = isStatic;
		return r;
	}
}