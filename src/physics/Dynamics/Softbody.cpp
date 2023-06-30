#include "physics/Dynamics/Softbody.hpp"
#include <iostream>

namespace physics
{
	f64 Spring::ForceExerting() const noexcept
	{
		f64 Fs = (geo::Distance(a->position, b->position) - restingLength) * stiffness;
		f64 Fd = (b->position - a->position).Normalized().Dot(b->velocity - a->velocity) * dampingFactor;
		return Fs + Fd;
	}

	bool Spring::operator==(const Spring& other) const noexcept
	{
		return restingLength == other.restingLength && dampingFactor == other.dampingFactor &&
			stiffness == other.stiffness;
	}

	bool Spring::operator!=(const Spring& other) const noexcept
	{
		return restingLength != other.restingLength || dampingFactor != other.dampingFactor ||
			stiffness != other.stiffness;
	}

	MassPoint::MassPoint()
	{
	}

	MassPoint::MassPoint(geo::Vector2 position, geo::Vector2 velocity, geo::Vector2 force, f64 invMass, f64 radius) noexcept
	: position(position), velocity(velocity), force(force), radius(radius), invMass(invMass)
	{
	}

	bool MassPoint::operator==(const MassPoint& other) const noexcept
	{
		return position == other.position && velocity == other.velocity &&
			force == other.force && invMass == other.invMass && radius == other.radius;
	}
	
	bool MassPoint::operator!=(const MassPoint& other) const noexcept
	{
		return position != other.position || velocity != other.velocity ||
			force != other.force || invMass != other.invMass || radius != other.radius;
	}
	

	Softbody::Softbody() noexcept
	: Dynamicbody(), width(0), height(0), radiusPerPoint(1)
	{
	}

	Softbody::Softbody(const Softbody& s) noexcept
	: Dynamicbody((const Dynamicbody&) s),
	width(s.width), height(s.height), radiusPerPoint(s.radiusPerPoint)
	{
		_isDynamic = true;
		for (auto vec: s.points)
		{
			points.push_back(vec);
		}
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				Spring spr = s.springs.size() ? s.springs[0] : Spring();
				spr.a = &points.at(i).at(j);
				if (i)
				{
					spr.b = &points.at(i - 1).at(j);
					springs.push_back(spr);
				}
				if (j != width - 1)
				{
					spr.b = &points.at(i).at(j + 1);
					springs.push_back(spr);
				}
				spr.restingLength = (sqrt(2) * spr.restingLength);
				if (i + 1 < height && j + 1 < width)
				{
					spr.b = &points.at(i + 1).at(j + 1);
					springs.push_back(spr);
				}
				if (i - 1 >= 0 && j + 1 < width)
				{
					spr.b = &points.at(i - 1).at(j + 1);
					springs.push_back(spr);
				}
			}
		}
	}

	Softbody::Softbody(Softbody && s) noexcept
	: Dynamicbody((Dynamicbody &&) s), points(s.points), springs(s.springs),
	width(s.width), height(s.height), radiusPerPoint(s.radiusPerPoint)
	{
	}

	Softbody::Softbody(const Transform& t, int width, int height, const Spring& spring, const f64& spacing,
		const f64& radiusPerPoint, const f64& invMassPerPoint) noexcept
	: Dynamicbody(BoxCollider(), t, false), width(width), height(height), radiusPerPoint(radiusPerPoint)
	{
		geo::Vector2 vec(0, height * spacing);
		for (int i = 0; i < height; i++)
		{
			std::vector<MassPoint> arr;
			for (int j = 0; j < width; j++)
			{
				MassPoint m;
				m.position = vec;
				m.invMass = invMassPerPoint;
				m.radius = radiusPerPoint;
				arr.push_back(m);
				vec.x += spacing;
			}
			points.push_back(arr);
			vec.x = 0;
			vec.y -= spacing;
		}
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				Spring s;
				s.stiffness = spring.stiffness;
				s.restingLength = spacing;
				s.dampingFactor = spring.dampingFactor;
				s.a = &points.at(i).at(j);
				if (i)
				{
					s.b = &points.at(i - 1).at(j);
					springs.push_back(s);
				}
				if (j != width - 1)
				{
					s.b = &points.at(i).at(j + 1);
					springs.push_back(s);
				}
				s.restingLength = (sqrt(2) * spacing);
				if (i + 1 < height && j + 1 < width)
				{
					s.b = &points.at(i + 1).at(j + 1);
					springs.push_back(s);
				}
				if (i - 1 >= 0 && j + 1 < width)
				{
					s.b = &points.at(i - 1).at(j + 1);
					springs.push_back(s);
				}
			}
		}
		MeshCollider m;
		for (int i = 0; i < height - 1; i++)
		{
			for (int j = 0; j < width - 1; j++)
			{
				geo::Vector2 a = points[i][j].position;
				geo::Vector2 b = points[i][j + 1].position;
				geo::Vector2 c = points[i + 1][j + 1].position;
				geo::Vector2 d = points[i + 1][j].position;
				PolygonCollider p(a, b, c, {d});
				_colliders.push_back(p);
			}
		}
		for (size_t i = 0; i < _colliders.size(); i++)
		{
			m.colliders.push_back(&_colliders.at(i));
		}
		SetCollider(m);
	}

	Softbody& Softbody::operator=(const Softbody& s) noexcept
	{
		Dynamicbody::operator=((const Dynamicbody&) s);
		radiusPerPoint = s.radiusPerPoint;
		points = s.points;
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				Spring spr = s.springs.size() ? s.springs[0] : Spring();
				spr.a = &points.at(i).at(j);
				if (i)
				{
					spr.b = &points.at(i - 1).at(j);
					springs.push_back(spr);
				}
				if (j != width - 1)
				{
					spr.b = &points.at(i).at(j + 1);
					springs.push_back(spr);
				}
				spr.restingLength = (sqrt(2) * spr.restingLength);
				if (i + 1 < height && j + 1 < width)
				{
					spr.b = &points.at(i + 1).at(j + 1);
					springs.push_back(spr);
				}
				if (i - 1 >= 0 && j + 1 < width)
				{
					spr.b = &points.at(i - 1).at(j + 1);
					springs.push_back(spr);
				}
			}
		}
		return *this;
	}

	Softbody& Softbody::operator=(Softbody && s) noexcept
	{
		Dynamicbody::operator=((Dynamicbody &&)s);
		radiusPerPoint = s.radiusPerPoint;
		points = s.points;
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				Spring spr = s.springs.size() ? s.springs[0] : Spring();
				spr.a = &points.at(i).at(j);
				if (i)
				{
					spr.b = &points.at(i - 1).at(j);
					springs.push_back(spr);
				}
				if (j != width - 1)
				{
					spr.b = &points.at(i).at(j + 1);
					springs.push_back(spr);
				}
				spr.restingLength = (sqrt(2) * spr.restingLength);
				if (i + 1 < height && j + 1 < width)
				{
					spr.b = &points.at(i + 1).at(j + 1);
					springs.push_back(spr);
				}
				if (i - 1 >= 0 && j + 1 < width)
				{
					spr.b = &points.at(i - 1).at(j + 1);
					springs.push_back(spr);
				}
			}
		}
		return *this;
	}

	bool Softbody::operator==(const CollisionObject& other) const noexcept
	{
		if (typeid(other).name() != typeid(*this).name())
			return false;
		auto o = dynamic_cast<const Softbody&>(other);
		return Dynamicbody::operator==((const Dynamicbody&)other) && (o.points == this->points) && 
			(o.springs == this->springs) && (o.width == this->width) &&
			(o.height == this->height) && (o.usesGravity == this->usesGravity);
	}

	bool Softbody::operator!=(const CollisionObject& other) const noexcept
	{
		if (typeid(other).name() != typeid(*this).name())
			return true;
		auto o = dynamic_cast<const Softbody&>(other);
		return Dynamicbody::operator!=(other) || (o.points != this->points) || 
			(o.springs != this->springs) || (o.width != this->width) ||
			(o.height != this->height) || (o.usesGravity != this->usesGravity);
	}

	void Softbody::ApplySpringForces(f64 dt) noexcept
	{
		for (Spring& s: springs)
		{
			s.a->force += s.ForceExerting() * (s.b->position - s.a->position).Normalized() * dt;
			s.b->force += s.ForceExerting() * (s.a->position - s.b->position).Normalized() * dt;	
			if ((s.a->force * s.a->invMass).GetMagnitudeSquared() > SQRD(EPSILON) ||( s.b->force * s.b->invMass).GetMagnitudeSquared() > SQRD(EPSILON))
				pointsChanged = true;
		}
	}

	void Softbody::ApplyImpulse(f64 dt, const geo::Vector2& impulse, const geo::Vector2& contactVec) noexcept
	{
		if (contactVec == geo::Vector2::Infinity)
		{
			for (auto& vec: points)
			{
				for (MassPoint& m: vec)
					m.velocity += impulse * m.invMass *  dt;
			}
		}
		else
		{
			for (auto& vec: points)
			{
				for (MassPoint& m: vec)
				{
					if (geo::DistanceSquared(transform.TransformVector(m.position), contactVec) <= SQRD(EPSILON))
					{
						m.velocity += impulse * m.invMass * dt;
						pointsChanged = true;
						return;
					}
				}
			}
		}
	}

	void Softbody::ApplyForce(f64 dt, const geo::Vector2& force, const geo::Vector2& contactVec) noexcept
	{
		if (contactVec == geo::Vector2::Infinity)
		{
			for (auto& vec: points)
			{
				for (MassPoint& m: vec)
					m.force += force *  dt;
			}
		}
		else
		{
			for (auto& vec: points)
			{
				for (MassPoint& m: vec)
				{
					if (geo::DistanceSquared(transform.TransformVector(m.position), contactVec) <= SQRD(EPSILON))
					{
						m.force += force * dt;
						pointsChanged = true;
						return;
					}
				}
			}
		}
	}

	void Softbody::ApplyAngularForce(f64 dt, f64 force) noexcept
	{
		angularForce += force * dt;
	}

	void Softbody::ApplyAngularImpulse(f64 dt, f64 force) noexcept
	{
		angularVelocity += force * dt;
	}

	CollisionObject* Softbody::Clone() const noexcept
	{
		return (CollisionObject*)new Softbody(*this);
	}

	void Softbody::FixCollapsing() noexcept
	{
		for (Spring& s: springs)
		{
			f64 dis = geo::Distance(s.a->position, s.b->position);
			if (!dis)
				continue;
			if (dis < s.a->radius + s.b->radius)
			{
				s.a->position += (s.b->position - s.a->position).Normalized() * ((s.a->radius + s.b->radius) - dis);
				s.a->velocity.Reflect((s.b->position - s.a->position).Normalized());
			}
		}
	}

	void Softbody::Update(f64 dt) noexcept
	{
		if (!isStatic)
		{
			velocity += (force * _invMass) * (dt / 2.0);
			angularVelocity += (angularForce * _invInertia) * (dt / 2.0);
			position += velocity * dt;
			rotation = geo::Matrix2(transform.GetAngle() + angularVelocity * dt);
			angularForce = 0;
			force.Set(0, 0);
			ApplySpringForces(dt);
			for (std::vector<MassPoint>& mVec: points)
			{
				for (MassPoint& m: mVec)
				{
					m.velocity += m.force * m.invMass * dt;
					m.position += m.velocity * dt;
					m.force.Set(0, 0);
				}
			}
			if (pointsChanged)
				UpdateCollider();
		}
	}

	void Softbody::UpdateCollider() noexcept
	{
		MeshCollider m;
		_colliders.clear();
		for (int i = 0; i < height - 1; i++)
		{
			for (int j = 0; j < width - 1; j++)
			{
				geo::Vector2 a = points[i][j].position;
				geo::Vector2 b = points[i][j + 1].position;
				geo::Vector2 c = points[i + 1][j + 1].position;
				geo::Vector2 d = points[i + 1][j].position;
				PolygonCollider p(geo::Vector2(0, 0), a, b, c, {d});
				_colliders.push_back(p);
			}
		}
		for (size_t i = 0; i < _colliders.size(); i++)
		{
			m.colliders.push_back(&_colliders.at(i));
		}
		SetCollider(m);
		pointsChanged = false;
	}
}
