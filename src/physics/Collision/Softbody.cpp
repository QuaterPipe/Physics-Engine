#include "../../include/physics/Collision/Softbody.hpp"
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

	MassPoint::MassPoint(geo::Vector position, geo::Vector velocity, geo::Vector force, f64 mass, f64 radius) noexcept
	: position(position), velocity(velocity), force(force), mass(mass), radius(radius)
	{
	}

	bool MassPoint::operator==(const MassPoint& other) const noexcept
	{
		return position == other.position && velocity == other.velocity &&
			force == other.force && mass == other.mass && radius == other.radius;
	}
	
	bool MassPoint::operator!=(const MassPoint& other) const noexcept
	{
		return position != other.position || velocity != other.velocity ||
			force != other.force || mass != other.mass || radius != other.radius;
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

	Softbody::Softbody(const Transform& t, unsigned width, unsigned height, const Spring& spring, const f64& spacing,
		const f64& radiusPerPoint, const f64& massPerPoint) noexcept
	: Dynamicbody(BoxCollider(), t, false), width(width), height(height), radiusPerPoint(radiusPerPoint)
	{
		geo::Vector vec(0, height * spacing);
		for (unsigned i = 0; i < height; i++)
		{
			std::vector<MassPoint> arr;
			for (unsigned j = 0; j < width; j++)
			{
				MassPoint m;
				m.position = vec;
				m.mass = massPerPoint;
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
				geo::Vector a = points[i][j].position;
				geo::Vector b = points[i][j + 1].position;
				geo::Vector c = points[i + 1][j + 1].position;
				geo::Vector d = points[i + 1][j].position;
				PolygonCollider p(a, b, c, {d});
				_colliders.push_back(p);
			}
		}
		for (unsigned i = 0; i < _colliders.size(); i++)
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
		UpdateCollider();
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
		UpdateCollider();
		return *this;
	}

	void Softbody::ApplyForce(const geo::Vector& Force, const geo::Vector& contactPoint) noexcept
	{
		if (contactPoint == geo::Vector::Infinity)
		{
			for (auto& vec: points)
			{
				for (MassPoint& v: vec)
					v.force += Force / v.mass;
			}
		}
		else
		{
			for (auto& vec: points)
			{
				for (MassPoint& m: vec)
				{
					if (geo::DistanceSquared(m.position, contactPoint) <= SQRD(0.000001))
					{
						m.force += Force / m.mass;
						break;
					}
				}
			}
		}
	}

	void Softbody::ApplySpringForces() noexcept
	{
		for (Spring& s: springs)
		{
			s.a->force += s.ForceExerting() * (s.b->position - s.a->position).Normalized();
			s.b->force += s.ForceExerting() * (s.a->position - s.b->position).Normalized();	
		}
	}

	void Softbody::ApplyImpulse(const geo::Vector& impulse, const geo::Vector& contactVec) noexcept
	{
	}

	void Softbody::ApplyAngularForce(f64 angularVelocity) noexcept
	{
	}

	CollisionObject* Softbody::Clone() const noexcept
	{
		return (CollisionObject*)new Softbody(*this);
	}

	bool Softbody::Equals(const Softbody& other) const noexcept
	{
		return Dynamicbody::Equals(other) && (other.points == this->points) && 
			(other.springs == this->springs) && (other.width == this->width) &&
			(other.height == this->height) && (other.usesGravity == this->usesGravity);
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

	bool Softbody::NotEquals(const Softbody& other) const noexcept
	{
		return Dynamicbody::NotEquals(other) || (other.points != this->points) || 
			(other.springs != this->springs) || (other.width != this->width) ||
			(other.height != this->height) || (other.usesGravity != this->usesGravity);
	}

	void Softbody::Update(f64 dt) noexcept
	{
		if (!isStatic)
		{
			for (std::vector<MassPoint>& mVec: points)
			{
				for (MassPoint& m: mVec)
				{
					m.velocity += m.force * (!m.mass ? 0 : 1 / m.mass) * dt;
					m.position += m.velocity * dt;
					m.force.Set(0, 0);
				}
			}
			UpdateCollider();
		}
	}

	void Softbody::UpdateCollider() noexcept
	{
		MeshCollider m;
		_colliders.clear();
		for (unsigned i = 0; i < height - 1; i++)
		{
			for (unsigned j = 0; j < width - 1; j++)
			{
				geo::Vector a = points[i][j].position;
				geo::Vector b = points[i][j + 1].position;
				geo::Vector c = points[i + 1][j + 1].position;
				geo::Vector d = points[i + 1][j].position;
				PolygonCollider p(geo::Vector(0, 0), a, b, c, {d});
				_colliders.push_back(p);
			}
		}
		for (unsigned i = 0; i < _colliders.size(); i++)
		{
			m.colliders.push_back(&_colliders.at(i));
		}
		SetCollider(m);
	}
}