#include "../../include/physics/Collision/Softbody.hpp"
#include <iostream>

namespace physics
{
	f64 Spring::ForceExerting() const noexcept
	{
		f64 Fs = (geometry::Distance(a->position, b->position) - restingLength) * stiffness;
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

	MassPoint::MassPoint(geometry::Vector position, geometry::Vector velocity, geometry::Vector force, f64 mass, f64 radius) noexcept
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
		geometry::Vector vec(0, height * spacing);
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
				geometry::Vector a = points[i][j].position;
				geometry::Vector b = points[i][j + 1].position;
				geometry::Vector c = points[i + 1][j + 1].position;
				geometry::Vector d = points[i + 1][j].position;
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

	void Softbody::ApplyForce(const geometry::Vector& force, const geometry::Vector& contactPoint) noexcept
	{
		if (contactPoint == geometry::Vector::Infinity)
		{
			for (auto& vec: points)
			{
				for (MassPoint& v: vec)
					v.force += force;
			}
		}
		else
		{
			for (auto& vec: points)
			{
				for (MassPoint& m: vec)
				{
					if (geometry::DistanceSquared(m.position, contactPoint) <= SQRD(0.000001))
					{
						m.force += force;
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

	void Softbody::ApplyImpulse(const geometry::Vector& impulse, const geometry::Vector& contactVec) noexcept
	{
	}

	void Softbody::ApplyAngularForce(f64 angularVelocity) noexcept
	{
	}

	CollisionObject* Softbody::Clone() const noexcept
	{
		return (CollisionObject*)new Softbody(*this);
	}

	bool Softbody::Equals(const Hashable& other) const noexcept
	{
		Softbody s;
		try
		{
			s = dynamic_cast<const Softbody&>(other);
		}
		catch(const std::bad_alloc& e)
		{
			return false;
		}
		return( s.points == this->points) && (s.springs == this->springs) && (s.width == this->width) &&
			(s.height == this->height) && (s.usesGravity == this->usesGravity);
	}

	void Softbody::FixCollapsing() noexcept
	{
		for (Spring& s: springs)
		{
			f64 dis = geometry::Distance(s.a->position, s.b->position);
			if (!dis)
				continue;
			if (dis < s.a->radius + s.b->radius)
			{
				s.a->position += (s.b->position - s.a->position).Normalized() * ((s.a->radius + s.b->radius) - dis);
				s.a->velocity.Reflect((s.b->position - s.a->position).Normalized());
			}
		}
	}

	bool Softbody::NotEquals(const Hashable& other) const noexcept
	{
		Softbody s;
		try
		{
			s = dynamic_cast<const Softbody&>(other);
		}
		catch(const std::bad_alloc& e)
		{
			return true;
		}
		return(s.points != this->points) || (s.springs != this->springs) || (s.width != this->width) ||
			(s.height != this->height) || (s.usesGravity != this->usesGravity);
	}

	void Softbody::UpdateCollider() noexcept
	{
		MeshCollider m;
		_colliders.clear();
		for (unsigned i = 0; i < height - 1; i++)
		{
			for (unsigned j = 0; j < width - 1; j++)
			{
				geometry::Vector a = points[i][j].position;
				geometry::Vector b = points[i][j + 1].position;
				geometry::Vector c = points[i + 1][j + 1].position;
				geometry::Vector d = points[i + 1][j].position;
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
}