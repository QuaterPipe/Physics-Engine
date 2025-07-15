#include <iostream>
#include "physics/Dynamics/PressureBody.hpp"

namespace physics
{
	Pressurebody::Pressurebody() noexcept
	{
	}

	Pressurebody::Pressurebody(f64 pressure, f64 radius, size_t pointCount, f64 mass, const PointMassSpring& sideSpring) noexcept
		: _pressureScalar(pressure), _radius(radius), _pointCount(pointCount), _pointStates(pointCount, RK4State()), _pressureForces(pointCount, geo::Vector2::Origin)
	{
		this->_mass = mass;
		this->_invMass = 1 / mass;
		if (pointCount < 3)
			return;
		f64 invMassPerPoint = pointCount / mass;
		geo::Vector2 dir(0, 1);
		const f64 rot = (2 * M_PI) / pointCount;
		for (size_t i = 0; i < _pointCount; i++)
		{
			PointMass m;
			m.position = dir * radius;
			m.invMass = invMassPerPoint;
			_points.push_back(m);
			dir.Rotate(geo::Vector2::Origin, rot);
		}
		for (size_t i = 0; i < _pointCount; i++)
		{
			PointMassSpring s = sideSpring;
			s.a = &_points[i];
			s.b = &_points[(i + 1) % _pointCount];
			_points[i].AddSpring(s, true);
			_points[(i + 1) % _pointCount].AddSpring(s, false);
			_springs.push_back(s);
		}
		f64 dis = geo::Distance(_points[0].position, _points[1].position);
		_collider = PolygonCollider(dis, pointCount);
	}

	Pressurebody::Pressurebody(const Pressurebody& p) noexcept
		: Dynamicbody(p)
	{
		_pressureScalar = p._pressureScalar;
		_radius = p._radius;
		_points = p._points;
		_springs = p._springs;
		_pointCount = p._pointCount;
		_pointStates = p._pointStates;
		_pressureForces = p._pressureForces;
		_collider = p._collider;
	}

	Pressurebody::Pressurebody(Pressurebody&& p) noexcept
		: Dynamicbody(p)
	{
		_pressureScalar = p._pressureScalar;
		_radius = p._radius;
		_points = p._points;
		_springs = p._springs;
		_pointCount = p._pointCount;
		_pointStates = p._pointStates;
		_pressureForces = p._pressureForces;
		_collider = p._collider;
	}

	Pressurebody& Pressurebody::operator=(const Pressurebody& p) noexcept
	{
		Dynamicbody::operator=(p);
		_pressureScalar = p._pressureScalar;
		_radius = p._radius;
		_points = p._points;
		_pointCount = p._pointCount;
		_springs = p._springs;
		_pointStates = p._pointStates;
		_pressureForces = p._pressureForces;
		_collider = p._collider;
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
		_pointStates = p._pointStates;
		_pressureForces = p._pressureForces;
		_collider = p._collider;
		return *this;
	}

	bool Pressurebody::operator==(const CollisionObject& other) const noexcept
	{
		if (typeid(other).name() != typeid(*this).name())
			return false;
		auto o = dynamic_cast<const Pressurebody&>(other);
		return Dynamicbody::operator==(o) && _radius == o._radius &&
			_points == o._points && _springs == o._springs &&
			_pointCount == o._pointCount && _pressureScalar == o._pressureScalar &&
			_collider == o._collider;
	}

	bool Pressurebody::operator!=(const CollisionObject& other) const noexcept
	{
		if (typeid(other).name() != typeid(*this).name())
			return true;
		auto o = dynamic_cast<const Pressurebody&>(other);
		return Dynamicbody::operator!=(o) || _radius != o._radius ||
			_points != o._points || _springs != o._springs ||
			_pointCount != o._pointCount || _pressureScalar == o._pressureScalar ||
			_collider == o._collider;
	}

	void Pressurebody::_UpdatePressureForces(int rk4step) noexcept
	{
		f64 volume = GetVolume(rk4step);
		for (geo::Vector2& p : _pressureForces)
			p.Set(0, 0);
		if (!rk4step)
		{
			for (size_t i = 0; i < _pointCount; i++)
			{
				geo::Vector2 a = _points[i].position;
				geo::Vector2 b = _points[(i + 1) % _pointCount].position;
				f64 dis = geo::Distance(a, b);
				geo::Vector2 norm(-(a.y - b.y) / dis, (a.x - b.x) / dis);
				f64 pressurev = dis * _pressureScalar * (1.0 / volume);
				_pressureForces[i] += norm * pressurev;
				_pressureForces[(i + 1) % _pointCount] += norm * pressurev;
			}
		}
		else
		{
			for (size_t i = 0; i < _pointCount; i++)
			{
				geo::Vector2 a = _pointStates[i].tmpX;
				geo::Vector2 b = _pointStates[(i + 1) % _pointCount].tmpX;
				f64 dis = geo::Distance(a, b);
				geo::Vector2 norm(-(a.y - b.y) / dis, (a.x - b.x) / dis);
				f64 pressurev = dis * _pressureScalar * (1.0 / volume);
				_pressureForces[i] += norm * pressurev * _points[i].invMass;
				_pressureForces[(i + 1) % _pointCount] += norm * pressurev * _points[i].invMass;
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

	void Pressurebody::ApplyForce(const geo::Vector2& Force, const geo::Vector2& contactPoint) noexcept
	{
		if (contactPoint == geo::Vector2::Infinity)
		{
			for (auto& m : _points)
				m.force += Force;
		}
		else
		{
			for (auto& m : _points)
			{
				if (geo::DistanceSquared(transform.TransformVector(m.position), contactPoint) <= SQRD(EPSILON))
				{	
					m.force += Force;
					return;
				}
			}
		}
	}

	void Pressurebody::ApplyImpulse(const geo::Vector2& impulse, const geo::Vector2 & contactVec) noexcept
	{
		if (contactVec == geo::Vector2::Infinity)
		{
			for (auto& m : _points)
				m.velocity += impulse* m.invMass;
		}
		else
		{
			for (auto& m : _points)
			{
				if (geo::DistanceSquared(transform.TransformVector(m.position), contactVec) <= SQRD(EPSILON))
				{
					m.velocity += impulse * m.invMass;
					return;
				}
			}
		}
	}

	CollisionObject* Pressurebody::Clone() const noexcept
	{
		return new Pressurebody(*this);
	}

	geo::Vector2 Pressurebody::ComputeForce(const geo::Vector2& position, const geo::Vector2& Velocity) const noexcept
	{
		return geo::Vector2(0, 0);
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
				return volume;
			case 0:
				for (int i = 0; i < _pointCount; i++)
				{
					volume += (_points[i].position.x * _points[(i + 1) % _pointCount].position.y) -
						(_points[(i + 1) % _pointCount].position.x * _points[i].position.y);
				}
				return volume;
			default:
				for (int i = 0; i < _pointCount; i++)
				{
					geo::Vector2 a = _pointStates[i].tmpX;
					geo::Vector2 b = _pointStates[(i + 1) % _pointCount].tmpX;
					volume += (a.x * b.y) - (b.x * a.y);
				}
				return volume;
		}
	}

	void Pressurebody::Update(f64 dt, int rk4step) noexcept
	{
		if (rk4step == 0)
		{
			_UpdatePressureForces(rk4step);
			for (size_t i = 0; i < _pointCount; i++)
			{
				_pointStates[i].a1 = _points[i].ComputeForce(_points[i].position, _points[i].velocity) + _pressureForces[i];
				_pointStates[i].k1X = _points[i].velocity;
				_pointStates[i].k1V = _pointStates[i].a1;
			}
		}
		else if (rk4step == 1)
		{
			for (size_t i = 0; i < _pointCount; i++)
			{
				_pointStates[i].tmpX = _points[i].position + 0.5 * dt * _pointStates[i].k1X;
				_pointStates[i].tmpV = _points[i].velocity + 0.5 * dt * _pointStates[i].k1V;
			}
			_UpdatePressureForces(rk4step);
			for (size_t i = 0; i < _pointCount; i++)
			{
				_pointStates[i].a2 = _points[i].ComputeForce(_pointStates[i].tmpX, _pointStates[i].tmpV) + _pressureForces[i];
				_pointStates[i].k2X = _pointStates[i].tmpV;
				_pointStates[i].k2V = _pointStates[i].a2;
			}
		}
		else if (rk4step == 2)
		{
			for (size_t i = 0; i < _pointCount; i++)
			{
				_pointStates[i].tmpX = _points[i].position + 0.5 * dt * _pointStates[i].k2X;
				_pointStates[i].tmpV = _points[i].velocity + 0.5 * dt * _pointStates[i].k2V;
			}
			_UpdatePressureForces(rk4step);
			for (size_t i = 0; i < _pointCount; i++)
			{
				_pointStates[i].a3 = _points[i].ComputeForce(_pointStates[i].tmpX, _pointStates[i].tmpV) + _pressureForces[i];
				_pointStates[i].k3X = _pointStates[i].tmpV;
				_pointStates[i].k3V = _pointStates[i].a3;
			}
		}
		else if (rk4step == 3)
		{
			for (size_t i = 0; i < _pointCount; i++)
			{
				_pointStates[i].tmpX = _points[i].position + dt * _pointStates[i].k3X;
				_pointStates[i].tmpV = _points[i].velocity + dt * _pointStates[i].k3V;
			}
			_UpdatePressureForces(rk4step);
			for (size_t i = 0; i < _pointCount; i++)
			{
				_pointStates[i].a4 = _points[i].ComputeForce(_pointStates[i].tmpX, _pointStates[i].tmpV) + _pressureForces[i];
				_pointStates[i].k4X = _pointStates[i].tmpV;
				_pointStates[i].k4V = _pointStates[i].a4;

				_points[i].position += (dt / 6.0) * (_pointStates[i].k1X + 2 * _pointStates[i].k2X + 2 * _pointStates[i].k3X + _pointStates[i].k4X);
				_points[i].velocity += (dt / 6.0) * (_pointStates[i].k1V + 2 * _pointStates[i].k2V + 2 * _pointStates[i].k3V + _pointStates[i].k4V);
				_points[i].force.Set(0, 0);
			}
			UpdateTransform();
			UpdateCollider();
		}
	}

	void Pressurebody::UpdateCollider() noexcept
	{
		for (size_t i = 0; i < _pointCount; i++)
			_collider.SetPoint(i, _points[i].position);
	}

	void Pressurebody::UpdateTransform() noexcept
	{
		geo::Vector2 locAvg = transform.GetPosition() * _pointCount;
		for (size_t i = 0; i < _pointCount; i++)
			locAvg += _points[i].position;
		locAvg /= _pointCount;
		geo::Vector2 diff = locAvg - transform.GetPosition();
		for (size_t i = 0; i < _pointCount; i++)
			_points[i].position -= diff;
		transform.SetPosition(locAvg);
	}
}