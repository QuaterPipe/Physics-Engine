#include "physics/Dynamics/Softbody.hpp"
#include <iostream>
const f64 root2 = sqrt(2);

namespace physics
{
	Softbody::Softbody() noexcept
	: Dynamicbody(), radiusPerPoint(DEFAULT_POINTMASS_RADIUS)
	{
	}

	Softbody::Softbody(const Softbody& s) noexcept
	: Dynamicbody((const Dynamicbody&) s), _originalShape(s.GetOriginalShape()),
	radiusPerPoint(s.radiusPerPoint), shapeMatchingOn(s.shapeMatchingOn), points(s.points), springs(s.springs)
	{
	}

	Softbody::Softbody(Softbody && s) noexcept
	: Dynamicbody((Dynamicbody &&) s), _originalShape(s.GetOriginalShape()), points(s.points), springs(s.springs),
	radiusPerPoint(s.radiusPerPoint), shapeMatchingOn(s.shapeMatchingOn)
	{

	}

	Softbody::Softbody(const Transform& t, size_t width, size_t height, const PointMassSpring& referenceSpring, const PointMassSpring& shapeSpring,
		const f64& radiusPerPoint, const f64& invMassPerPoint) noexcept
	: Dynamicbody(BoxCollider(), t, false), radiusPerPoint(radiusPerPoint), pointCount(width * height), shapeSpring(shapeSpring)
	{
		const f64 spacing = referenceSpring.restingLength;
		Vector2 vec(width * spacing * -0.5, height * spacing * 0.5);
		for (size_t i = 0; i < height; i++)
		{
			std::vector<PointMass> arr;
			for (size_t j = 0; j < width; j++)
			{
				PointMass m;
				m.position = vec;
				m.invMass = invMassPerPoint;
				m.radius = radiusPerPoint;
				points.push_back(m);
				vec.x += spacing;
			}
			vec.x = width * spacing * -0.5;
			vec.y -= spacing;
		}
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				PointMassSpring s;
				s.stiffness = referenceSpring.stiffness;
				s.restingLength = spacing;
				s.dampingFactor = referenceSpring.dampingFactor;
				s.a = &points.at(i * width + j);
				s.aIndex = i * width + j;
				if (i)
				{
					s.b = &points.at((i - 1) * width + j);
					s.bIndex = (i - 1) * width + j;
					points.at(s.aIndex).AddSpring(s, true);
					points.at(s.bIndex).AddSpring(s, false);
					springs.push_back(s);
				}
				if (j != width - 1)
				{
					s.b = &points.at(i * width + j + 1);
					s.bIndex = i * width + j + 1;
					points.at(s.aIndex).AddSpring(s, true);
					points.at(s.bIndex).AddSpring(s, false);
					springs.push_back(s);
				}
				s.restingLength = (sqrt(2) * spacing);
				if (i + 1 < height && j + 1 < width)
				{
					s.b = &points.at((i + 1) * width + j + 1);
					s.bIndex = (i + 1) * width + j + 1;
					points.at(s.aIndex).AddSpring(s, true);
					points.at(s.bIndex).AddSpring(s, false);
					springs.push_back(s);
				}
				if (i - 1 >= 0 && j + 1 < width)
				{
					s.b = &points.at((i - 1) * width + j + 1);
					s.bIndex = (i - 1) * width + j + 1;
					points.at(s.aIndex).AddSpring(s, true);
					points.at(s.bIndex).AddSpring(s, false);
					springs.push_back(s);
				}
			}
		}
		UpdateTransform();
		_originalShape = points;
		for (size_t i = 0; i < pointCount; i++)
		{
			PointMassSpring s = shapeSpring;
			s.a = &points[i];
			s.b = &_originalShape[i];
			points[i].AddCorrectionSpring(s, true);
			points[i].correctionOn = shapeMatchingOn;
		}
		_pointStates = std::vector<RK4State>(pointCount);
	}

	Softbody& Softbody::operator=(const Softbody& s) noexcept
	{
		Dynamicbody::operator=((const Dynamicbody&) s);
		pointCount = s.pointCount;
		_originalShape = s.GetOriginalShape();
		shapeMatchingOn = s.shapeMatchingOn;
		radiusPerPoint = s.radiusPerPoint;
		points = s.points;
		springs = s.springs;
		shapeSpring = s.shapeSpring;
		for (size_t i = 0; i < pointCount; i++)
		{
			springs[i].a = &points[springs[i].aIndex];
			springs[i].b = &points[springs[i].bIndex];
		}
		_pointStates = std::vector<RK4State>(pointCount);
		return *this;
	}

	Softbody& Softbody::operator=(Softbody && s) noexcept
	{
		Dynamicbody::operator=((Dynamicbody &&)s);
		pointCount = s.pointCount;
		_originalShape = s.GetOriginalShape();
		shapeMatchingOn = s.shapeMatchingOn;
		radiusPerPoint = s.radiusPerPoint;
		points = s.points;
		springs = s.springs;
		shapeSpring = s.shapeSpring;
		for (size_t i = 0; i < pointCount; i++)
		{
			springs[i].a = &points[springs[i].aIndex];
			springs[i].b = &points[springs[i].bIndex];
		}
		_pointStates = std::vector<RK4State>(pointCount);
		return *this;
	}

	bool Softbody::operator==(const CollisionObject& other) const noexcept
	{
		if (typeid(other).name() != typeid(*this).name())
			return false;
		auto o = dynamic_cast<const Softbody&>(other);
		return Dynamicbody::operator==((const Dynamicbody&)other) && (o.points == this->points) && 
			(o.springs == this->springs) && (o.usesGravity == this->usesGravity) && (o.shapeMatchingOn == this->shapeMatchingOn)
			&& (o.GetOriginalShape() == _originalShape);
	}

	bool Softbody::operator!=(const CollisionObject& other) const noexcept
	{
		if (typeid(other).name() != typeid(*this).name())
			return true;
		auto o = dynamic_cast<const Softbody&>(other);
		return Dynamicbody::operator!=(other) || (o.points != this->points) || 
			(o.springs != this->springs) || (o.usesGravity != this->usesGravity) || (o.shapeMatchingOn != this->shapeMatchingOn)
			|| (o.GetOriginalShape() != _originalShape);
	}

	void Softbody::ApplyImpulse(const Vector2& impulse, const Vector2& contactVec) noexcept
	{
		if (contactVec == Vector2::Infinity)
		{
			for (auto& m: points)
			{
				m.velocity += impulse * m.invMass;
			}
		}
		else
		{
			for (auto& m: points)
			{
				if (DistanceSquared(transform.TransformVector(m.position), contactVec) <= SQRD(EPSILON))
				{
					m.velocity += impulse * m.invMass;
					_pointsChanged = true;
					return;
				}
			}
		}
	}

	void Softbody::ApplyForce(const Vector2& force, const Vector2& contactVec) noexcept
	{
		if (contactVec == Vector2::Infinity)
		{
			for (auto& m: points)
			{
				m.force += force;
			}
		}
		else
		{
			for (auto& m: points)
			{
				if (DistanceSquared(transform.TransformVector(m.position), contactVec) <= SQRD(EPSILON))
				{
					m.force += force;
					_pointsChanged = true;
					return;
				}
			}
		}
	}

	void Softbody::ApplyAngularForce(f64 force) noexcept
	{
		appliedAngularForce += force;
	}

	void Softbody::ApplyAngularImpulse(f64 force) noexcept
	{
		angularVelocity += force * _invInertia;
	}

	CollisionObject* Softbody::Clone() const noexcept
	{
		return (CollisionObject*)new Softbody(*this);
	}

	f64 Softbody::ComputeAngularForce(f64 orient, f64 angVelocity) const noexcept
	{
		return appliedAngularForce;
	}


	Vector2 Softbody::ComputeForce(const Vector2& position, const Vector2& velocity, f64 orient) const noexcept
	{
		return appliedForce;
	}


	void Softbody::DerivePositionAndAngle() noexcept
	{
		derivedPos.Set(0, 0);
		for (size_t i = 0; i < pointCount; i++)
		{
			derivedPos += points[i].position;
			derivedVel += points[i].velocity;
		}
		derivedPos /= pointCount;
		f64 a = 0, b = 0;
		for (size_t i = 0; i < pointCount; i++)
		{
			Vector2 q0 = _originalShape[i].position - _originalCenter;
			Vector2 q = points[i].position - derivedPos;
			a += q.Dot(q0);
			b += q0.Cross(q);
		}
		derivedAngle = atan2(b, a);

	}

	void Softbody::FixCollapsing() noexcept
	{
		for (PointMassSpring& s: springs)
		{
			f64 dis = DistanceSquared(s.a->position, s.b->position);
			if (!dis)
				continue;
			if (dis < SQRD(s.a->radius + s.b->radius))
			{
				s.a->position += (s.b->position - s.a->position).Normalized() * ((s.a->radius + s.b->radius) - dis);
				s.a->velocity.Reflect((s.b->position - s.a->position).Normalized());
			}
		}
	}

	PointMass* Softbody::GetClosestMassPoint(const Vector2& point) const noexcept
	{
		PointMass* closest = nullptr;
		f64 closestDis = std::numeric_limits<f64>::infinity();
		for (auto& m : points)
		{
			const f64 dis = DistanceSquared(m.position + transform.GetPosition(), point);
			if (dis < closestDis)
			{
				closest = const_cast<PointMass*>(&m);
				closestDis = dis;
				if (closestDis <= EPSILON)
					return closest;
			}
		}
		return closest;
	}

	const std::vector<PointMass>& Softbody::GetOriginalShape() const noexcept
	{
		return _originalShape;
	}

	f64 Softbody::MassScaler() const noexcept
	{
		return pointCount;
	}


	void Softbody::Update(f64 dt, int RK4Step) noexcept
	{
		if (!isStatic)
		{
			switch (RK4Step)
			{
				case 0:
					posState.Reset();
					for (size_t i = 0; i < pointCount; i++)
					{
						_pointStates[i].a1 = points[i].ComputeForce(points[i].position, points[i].velocity);
						_pointStates[i].k1X = points[i].velocity;
						_pointStates[i].k1V = _pointStates[i].a1;
						posState.a1 += _pointStates[i].a1;
						posState.k1X += _pointStates[i].k1X;
						posState.k1V += _pointStates[i].k1V;
					}
					posState.a1 /= pointCount;
					posState.k1X /= pointCount;
					posState.k1V /= pointCount;
					break;
				case 1:
					for (size_t i = 0; i < pointCount; i++)
					{
						_pointStates[i].tmpX = points[i].position + 0.5 * dt * _pointStates[i].k1X;
						_pointStates[i].tmpV = points[i].velocity + 0.5 * dt * _pointStates[i].k1V;
						_pointStates[i].a2 = points[i].ComputeForce(_pointStates[i].tmpX, _pointStates[i].tmpV);
						_pointStates[i].k2X = _pointStates[i].tmpV;
						_pointStates[i].k2V = _pointStates[i].a2;
						posState.a2 += _pointStates[i].a2;
						posState.k2X += _pointStates[i].k2X;
						posState.k2V += _pointStates[i].k2V;
						posState.tmpX += _pointStates[i].tmpX;
						posState.tmpV += _pointStates[i].tmpV;
					}
					posState.a1 /= pointCount;
					posState.k1X /= pointCount;
					posState.k1V /= pointCount;
					posState.tmpX /= pointCount;
					posState.tmpV /= pointCount;
					break;
				case 2:
					for (size_t i = 0; i < pointCount; i++)
					{
						_pointStates[i].tmpX = points[i].position + 0.5 * dt * _pointStates[i].k2X;
						_pointStates[i].tmpV = points[i].velocity + 0.5 * dt * _pointStates[i].k2V;
						_pointStates[i].a3 = points[i].ComputeForce(_pointStates[i].tmpX, _pointStates[i].tmpV);
						_pointStates[i].k3X = _pointStates[i].tmpV;
						_pointStates[i].k3V = _pointStates[i].a3;
						posState.a3 += _pointStates[i].a3;
						posState.k3X += _pointStates[i].k3X;
						posState.k3V += _pointStates[i].k3V;
						posState.tmpX += _pointStates[i].tmpX;
						posState.tmpV += _pointStates[i].tmpV;
					}
					posState.a1 /= pointCount;
					posState.k1X /= pointCount;
					posState.k1V /= pointCount;
					posState.tmpX /= pointCount;
					posState.tmpV /= pointCount;
					break;
				case 3:
					for (size_t i = 0; i < pointCount; i++)
					{
						_pointStates[i].tmpX = points[i].position + dt * _pointStates[i].k3X;
						_pointStates[i].tmpV = points[i].velocity + dt * _pointStates[i].k3V;
						_pointStates[i].a4 = points[i].ComputeForce(_pointStates[i].tmpX, _pointStates[i].tmpV);
						_pointStates[i].k4X = _pointStates[i].tmpV;
						_pointStates[i].k4V = _pointStates[i].a4;

						points[i].position += (dt / 6.0) * (_pointStates[i].k1X + 2 * _pointStates[i].k2X + 2 * _pointStates[i].k3X + _pointStates[i].k4X);
						points[i].velocity += (dt / 6.0) * (_pointStates[i].k1V + 2 * _pointStates[i].k2V + 2 * _pointStates[i].k3V + _pointStates[i].k4V);
						points[i].force.Set(0, 0);
					}
					DerivePositionAndAngle();
					UpdateTransform();
					if (_pointsChanged)
						UpdateCollider();
					FixCollapsing();
					break;
				default:
					break;
			}
		}
	}

	void Softbody::UpdateCollider() noexcept
	{
	}

	void Softbody::UpdateTransform() noexcept
	{
		Vector2 locAvg = transform.GetPosition() * pointCount;
		for (size_t i = 0; i < pointCount; i++)
		{
			locAvg += points[i].position;
		}
		locAvg /= pointCount;
		Vector2 diff = locAvg - transform.GetPosition();
		for (size_t i = 0; i < pointCount; i++)
		{
			points[i].position -= diff;
		}
		transform.SetPosition(locAvg);
	}
}
