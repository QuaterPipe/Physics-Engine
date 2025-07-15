#include "physics/Dynamics/Rigidbody.hpp"
#include "physics/Dynamics/Integration.hpp"


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

	void Rigidbody::ApplyForce(const geo::Vector2& Force, const geo::Vector2& contactPoint) noexcept
	{
		if (!isStatic && !isKinematic)
		{
		 	force += Force;
			if (contactPoint != geo::Vector2::Infinity && Force.GetMagnitudeExact())
			{
				angularForce += (transform.GetCOM()).Cross(Force);
			}
		}
	}

	void Rigidbody::ApplyImpulse(const geo::Vector2& impulse, const geo::Vector2& contactVec) noexcept
	{
		if (!isStatic && !isKinematic)
		{
			velocity += _invMass * impulse;
			if (contactVec != geo::Vector2::Infinity && impulse.GetMagnitudeExact())
				angularVelocity += _invInertia * contactVec.Cross(impulse);
		}
	}

	f64 Rigidbody::ComputeAngularForce(f64 orientation, f64 angVelocity) const noexcept
	{
		return appliedAngularForce;
	}

	geo::Vector2 Rigidbody::ComputeForce(const geo::Vector2& position, const geo::Vector2 &velocity) const noexcept
	{
		return appliedForce + (usesGravity ? gravity : geo::Vector2(0, 0));
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

	void Rigidbody::Update(f64 dt, int RK4Step) noexcept
	{
		if (!isKinematic && !isStatic)
		{
			geo::Vector2 position = transform.GetPosition();
			f64 orient = transform.GetAngle();
			switch (RK4Step)
			{
				case 0:
					_state.a1 = ComputeForce(position, velocity);
					_state.k1X = velocity;
					_state.k1V = _state.a1;

					_angState.a1.x = ComputeAngularForce(orient, angularVelocity);
					_angState.k1X.x = angularVelocity;
					_angState.k1V = _angState.a1;
					break;
				case 1:
					_state.tmpX = position + 0.5 * dt * _state.k1X;
					_state.tmpV = velocity + 0.5 * dt * _state.k1V;
					_state.a2 = ComputeForce(_state.tmpX, _state.tmpV);
					_state.k2X = _state.tmpV;
					_state.k2V = _state.a2;

					_angState.tmpX = orient + 0.5 * dt * _angState.k1X;
					_angState.tmpV = angularVelocity + 0.5 * dt * _angState.k1V;
					_angState.a2.x = ComputeAngularForce(_angState.tmpX.x, _angState.tmpV.x);
					_angState.k2X = _angState.tmpV;
					_angState.k2V = _angState.a2;
					break;
				case 2:
					_state.tmpX = position + 0.5 * dt * _state.k2X;
					_state.tmpV = velocity + 0.5 * dt * _state.k2V;
					_state.a3 = ComputeForce(_state.tmpX, _state.tmpV);
					_state.k3X = _state.tmpV;
					_state.k3V = _state.a3;

					_angState.tmpX = orient + 0.5 * dt * _angState.k2X;
					_angState.tmpV = angularVelocity + 0.5 * dt * _angState.k2V;
					_angState.a3.x = ComputeAngularForce(_angState.tmpX.x, _angState.tmpV.x);
					_angState.k3X = _angState.tmpV;
					_angState.k3V = _angState.a3;
					break;
				case 3:
					_state.tmpX = position + dt * _state.k3X;
					_state.tmpV = velocity + dt * _state.k3V;
					_state.a4 = ComputeForce(_state.tmpX, _state.tmpV);
					_state.k4X = _state.tmpV;
					_state.k4V = _state.a4;

					_angState.tmpX = orient + dt * _angState.k3X;
					_angState.tmpV = angularVelocity + dt * _angState.k3V;
					_angState.a4.x = ComputeAngularForce(_angState.tmpX.x, _angState.tmpV.x);
					_angState.k4X = _angState.tmpV;
					_angState.k4V = _angState.a4;

					position = position + (dt / 6.0) * (_state.k1X + 2 * _state.k2X + 2 * _state.k3X + _state.k4X);
					velocity = velocity + (dt / 6.0) * (_state.k1V + 2 * _state.k2V + 2 * _state.k3V + _state.k4V);

					orient = orient + (dt / 6.0) * (_angState.k1X.x + 2 * _angState.k2X.x + 2 * _angState.k3X.x + _angState.k4X.x);
					angularVelocity = angularVelocity + (dt / 6.0) * (_angState.k1V.x + 2 * _angState.k2V.x + 2 * _angState.k3V.x + _angState.k4V.x);
					
					transform.SetPosition(position);
					transform.SetAngle(orient);
					appliedAngularForce = 0;
					appliedForce = geo::Vector2(0, 0);
					break;
				default:
					break;
			}
			/*geo::Vector2 position = transform.GetPosition();
			SymplecticEulerIntegrate(&position.x, &velocity.x, &force.x, dt);
			SymplecticEulerIntegrate(&position.y, &velocity.y, &force.y, dt);
			f64 ang = transform.GetAngle();
			SymplecticEulerIntegrate(&ang, &angularVelocity, &angularForce, dt);
			transform.SetAngle(ang);
			transform.SetPosition(position);
			angularForce = 0;
			force.Set(0, 0);*/
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