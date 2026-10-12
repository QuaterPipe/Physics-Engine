#include <iostream>
#include "physics/Dynamics/PressureBody.hpp"
#define DAMPING 1
namespace physics
{
	Pressurebody::Pressurebody() noexcept
	{
	}

	Pressurebody::Pressurebody(f64 pressure, f64 radius, size_t pointCount, f64 mass, const PointMassSpring& sideSpring) noexcept
		: _pressureScalar(pressure), _radius(radius), _pointCount(pointCount), _pressureForces(pointCount, Vector2::Origin)
	{
		this->_mass = mass;
		this->_invMass = 1 / mass;
		this->_invInertia = 0;
		if (pointCount < 3) // not a valid polygon
			return;
		f64 invMassPerPoint = _invMass * pointCount;
		Vector2 dir(0, 1);
		const f64 rot = (2 * M_PI) / pointCount;
		Vector2 sum(0, 0);
		_points.resize(_pointCount);
		for (size_t i = 0; i < _pointCount; i++)
		{
			PointMass m;
			m.position = dir * radius;
			sum += m.position;
			m.invMass = invMassPerPoint;
			_points[i] = m;
			dir.Rotate(Vector2::Origin, rot);
		}
		f64 dist = (_points[0].position - _points[1].position).GetMagnitude();
		for (size_t i = 0; i < _pointCount; i++)
		{
			PointMassSpring s = sideSpring;
			s.a = &_points[i];
			s.b = &_points[(i + 1) % _pointCount];
			s.restingLength = dist;
			_points[i].AddSpring(s, true);
			_points[(i + 1) % _pointCount].AddSpring(s, false);
			_springs.push_back(s);
		}
		f64 dis = Distance(_points[0].position, _points[1].position);
		collider = std::unique_ptr<PolygonCollider>(new PolygonCollider(dis, pointCount));
	}

	Pressurebody::Pressurebody(const Pressurebody& p) noexcept
		: Dynamicbody(p)
	{
		_pressureScalar = p._pressureScalar;
		_radius = p._radius;
		_points = p._points;
		_springs = p._springs;
		_pointCount = p._pointCount;
		_pressureForces = p._pressureForces;
	}

	Pressurebody::Pressurebody(Pressurebody&& p) noexcept
		: Dynamicbody(p)
	{
		_pressureScalar = p._pressureScalar;
		_radius = p._radius;
		_points = p._points;
		_springs = p._springs;
		_pointCount = p._pointCount;
		_pressureForces = p._pressureForces;
	}

	Pressurebody& Pressurebody::operator=(const Pressurebody& p) noexcept
	{
		Dynamicbody::operator=(p);
		_pressureScalar = p._pressureScalar;
		_radius = p._radius;
		_points = p._points;
		_pointCount = p._pointCount;
		_springs = p._springs;
		_pressureForces = p._pressureForces;
		return *this;
	}

	Pressurebody& Pressurebody::operator=(Pressurebody&& p) noexcept
	{
		Dynamicbody::operator=((Dynamicbody&&)p);
		_pressureScalar = p._pressureScalar;
		_radius = p._radius;
		_points = p._points;
		_pointCount = p._pointCount;
		_springs = p._springs;
		_pressureForces = p._pressureForces;
		return *this;
	}

	bool Pressurebody::operator==(const CollisionObject& other) const noexcept
	{
		if (typeid(other).name() != typeid(*this).name())
			return false;
		auto o = dynamic_cast<const Pressurebody&>(other);
		return Dynamicbody::operator==(o) && _radius == o._radius &&
			_points == o._points && _springs == o._springs &&
			_pointCount == o._pointCount && _pressureScalar == o._pressureScalar;
	}

	bool Pressurebody::operator!=(const CollisionObject& other) const noexcept
	{
		if (typeid(other).name() != typeid(*this).name())
			return true;
		auto o = dynamic_cast<const Pressurebody&>(other);
		return Dynamicbody::operator!=(o) || _radius != o._radius ||
			_points != o._points || _springs != o._springs ||
			_pointCount != o._pointCount || _pressureScalar == o._pressureScalar;
	}

	void Pressurebody::_UpdatePressureForces(int rk4step) noexcept
	{
		f64 volume = GetVolume(rk4step);
		for (Vector2& p : _pressureForces)
			p.Set(0, 0);
		if (!rk4step)
		{
			for (size_t i = 0; i < _pointCount; i++)
			{
				Vector2 a = _points[i].position;
				Vector2 b = _points[(i + 1) % _pointCount].position;
				f64 dis = Distance(a, b);
				Vector2 norm(-(a.y - b.y) / dis, (a.x - b.x) / dis);
				f64 pressurev = dis * _pressureScalar * (1.0 / volume);
				_pressureForces[i] += DAMPING * norm * pressurev *_points[i].invMass;
				_pressureForces[(i + 1) % _pointCount] += DAMPING * norm * pressurev * _points[(i + 1) % _pointCount].invMass;
			}
		}
		else
		{
			for (size_t i = 0; i < _pointCount; i++)
			{
				Vector2 a = _points[i].rk4State.tmpX;
				Vector2 b = _points[(i + 1) % _pointCount].rk4State.tmpX;
				f64 dis = Distance(a, b);
				Vector2 norm(-(a.y - b.y) / dis, (a.x - b.x) / dis);
				f64 pressurev = dis * _pressureScalar * (1.0 / volume);
				_pressureForces[i] += norm * pressurev * _points[i].invMass;
				_pressureForces[(i + 1) % _pointCount] += norm * pressurev * _points[(i + 1) % _pointCount].invMass;
			}
		}
	}


	void Pressurebody::ApplyAngularForce(f64 Force) noexcept
	{
		appliedAngularForce += Force;
	}

	void Pressurebody::ApplyAngularImpulse(f64 impulse) noexcept
	{
		angularVelocity += impulse;
	}

	void Pressurebody::ApplyForce(const Vector2& Force, const Vector2& contactPoint) noexcept
	{
		if (contactPoint == Vector2::Infinity)
		{
			for (auto& m : _points)
				m.force += Force;
		}
		else
		{
			for (auto& m : _points)
			{
				if (DistanceSquared(transform.TransformVector(m.position), contactPoint) <= SQRD(EPSILON))
				{	
					m.force += Force;
					return;
				}
			}
		}
	}

	void Pressurebody::ApplyImpulse(const Vector2& impulse, const Vector2 & contactVec) noexcept
	{
		if (contactVec == Vector2::Infinity)
			for (auto& m : _points)
				m.velocity += impulse * m.invMass;
		else
		{
			for (auto& m : _points)
			{
				if (DistanceSquared(m.position, contactVec) <= SQRD(EPSILON))
				{
					m.velocity += impulse * m.invMass;
					return;
				}
			}
		}
		// for now any impulses that do not lie on a point are ignored.
	}

	CollisionObject* Pressurebody::Clone() const noexcept
	{
		return new Pressurebody(*this);
	}

	Vector2 Pressurebody::ComputeForce(const Vector2& position, const Vector2& Velocity, f64 orient) const noexcept
	{
		return Vector2::Origin;
	}

	void Pressurebody::DerivePositionAndAngle() noexcept
	{
		Vector2 derivedPos(0, 0);
		Vector2 derivedVel(0, 0);
		for (size_t i = 0; i < _pointCount; i++)
		{
			derivedPos += _points[i].position;
			derivedVel += _points[i].velocity;
		}
		//derivedVel /= _pointCount;
		derivedPos /= _pointCount;
		velocity = derivedVel;
	}

	f64 Pressurebody::GetRadius() const noexcept
	{
		return _radius;
	}

	const std::vector<PointMass>& Pressurebody::GetPoints() const noexcept
	{
		return _points;
	}

	f64 Pressurebody::GetPressureScalar() const noexcept
	{
		return _pressureScalar;
	}

	const std::vector<PointMassSpring>& Pressurebody::GetSprings() const noexcept
	{
		return _springs;
	}

	f64 Pressurebody::GetVolume(int rk4step) const noexcept
	{
		f64 volume = 0;
		switch (rk4step)
		{
			case -1:
				for (int i = 0; i < _pointCount; i++)
				{
					volume += (_points[i].position.x * _points[(i + 1) % _pointCount].position.y) -
						(_points[(i + 1) % _pointCount].position.x * _points[i].position.y);
				}
				return abs(volume) * 0.5;
			case 0:
				for (int i = 0; i < _pointCount; i++)
				{
					volume += (_points[i].position.x * _points[(i + 1) % _pointCount].position.y) -
						(_points[(i + 1) % _pointCount].position.x * _points[i].position.y);
				}
				return abs(volume) * 0.5;
			default:
				for (int i = 0; i < _pointCount; i++)
				{
					Vector2 a = _points[i].rk4State.tmpX;
					Vector2 b = _points[(i + 1) % _pointCount].rk4State.tmpX;
					volume += (a.x * b.y) - (b.x * a.y);
				}
				return abs(volume) * 0.5;
		}
	}

	f64 Pressurebody::MassScaler() const noexcept
	{
		return _pointCount;
	}

	void Pressurebody::Translate(const Vector2& offset, const std::vector<Vector2>& points) noexcept
	{
		for (auto p : points)
		{
			for (auto& m : _points)
			{
				if (DistanceSquared(transform.TransformVector(m.position), p) <= SQRD(EPSILON))
				{
					m.position += offset;
					continue;
				}
			}
		}
	}


	void Pressurebody::Update(f64 dt, int rk4step) noexcept
	{
		Vector2 grav = usesGravity ? gravity : Vector2(0, 0);
		switch (rk4step)
		{
			case 0:
				_UpdatePressureForces(rk4step);
				for (size_t i = 0; i < _pointCount; i++)
				{
					_points[i].rk4State.a1 = _points[i].ComputeForce(_points[i].position, _points[i].velocity, rk4step) + _pressureForces[i]
						+ grav;
					_points[i].rk4State.k1X = _points[i].velocity;
					_points[i].rk4State.k1V = _points[i].rk4State.a1;
				}
				break;

			case 1:
				for (size_t i = 0; i < _pointCount; i++)
				{
					_points[i].rk4State.tmpX = _points[i].position + 0.5 * dt * _points[i].rk4State.k1X;
					_points[i].rk4State.tmpV = _points[i].velocity + 0.5 * dt * _points[i].rk4State.k1V;
				}
				_UpdatePressureForces(rk4step);
				for (size_t i = 0; i < _pointCount; i++)
				{
					_points[i].rk4State.a2 = _points[i].ComputeForce(Vector2(0, 0), Vector2(0, 0), rk4step) + _pressureForces[i]
						+ grav;
					_points[i].rk4State.k2X = _points[i].rk4State.tmpV;
					_points[i].rk4State.k2V = _points[i].rk4State.a2;
				}
				break;
			
			case 2:
				for (size_t i = 0; i < _pointCount; i++)
				{
					_points[i].rk4State.tmpX = _points[i].position + 0.5 * dt * _points[i].rk4State.k2X;
					_points[i].rk4State.tmpV = _points[i].velocity + 0.5 * dt * _points[i].rk4State.k2V;
				}
				_UpdatePressureForces(rk4step);
				for (size_t i = 0; i < _pointCount; i++)
				{
					_points[i].rk4State.a3 = _points[i].ComputeForce(Vector2(0, 0), Vector2(0, 0), rk4step) + _pressureForces[i]
						+ grav;
					_points[i].rk4State.k3X = _points[i].rk4State.tmpV;
					_points[i].rk4State.k3V = _points[i].rk4State.a3;
				}
				break;
			
			case 3:
				for (size_t i = 0; i < _pointCount; i++)
				{
					_points[i].rk4State.tmpX = _points[i].position + dt * _points[i].rk4State.k3X;
					_points[i].rk4State.tmpV = _points[i].velocity + dt * _points[i].rk4State.k3V;
				}
				_UpdatePressureForces(rk4step);
				Vector2 minDiff(Vector2::Infinity());
				for (size_t i = 0; i < _pointCount; i++)
				{
					_points[i].rk4State.a4 = _points[i].ComputeForce(Vector2(0, 0), Vector2(0, 0), rk4step) + _pressureForces[i]
						+ grav;
					_points[i].rk4State.k4X = _points[i].rk4State.tmpV;
					_points[i].rk4State.k4V = _points[i].rk4State.a4;
					_points[i].position += (dt / 6.0) * (_points[i].rk4State.k1X + 2 * _points[i].rk4State.k2X + 2 * _points[i].rk4State.k3X + _points[i].rk4State.k4X);
					_points[i].velocity += (dt / 6.0) * (_points[i].rk4State.k1V + 2 * _points[i].rk4State.k2V + 2 * _points[i].rk4State.k3V + _points[i].rk4State.k4V);
					_points[i].force.Set(0, 0);
				}
				UpdateTransform();
				UpdateCollider();
				DerivePositionAndAngle();
				break;
		}
	}

	void Pressurebody::UpdateCollider() noexcept
	{
		for (size_t i = 0; i < _pointCount; i++)
			((PolygonCollider&)(*collider)).SetPoint(i, _points[i].position);
	}

	void Pressurebody::UpdateTransform() noexcept
	{
		Vector2 initialCOM(0, 0);
		for (auto p : _points)
			initialCOM += p.position;
		initialCOM /= _pointCount;
		transform.Translate(initialCOM);
		for (auto& p : _points)
			p.position -= initialCOM;
	}

	Vector2 Pressurebody::VelocityAt(const Vector2& vec) const noexcept
	{
		for (auto m : _points)
			if (DistanceSquared(m.position, vec) <= SQRD(EPSILON))
				return m.velocity;
		return velocity + Vector2::Cross(angularVelocity, vec);
	}

}