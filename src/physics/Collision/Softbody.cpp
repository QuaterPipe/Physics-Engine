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

	MassPoint::MassPoint()
	{
	}

	MassPoint::MassPoint(geometry::Vector position, geometry::Vector velocity, geometry::Vector force, f64 mass) noexcept
	{
		position = position;
		velocity = velocity;
		force = force;
		mass = mass;
	}

	Softbody::Softbody() noexcept
	: CollisionObject(), width(0), height(0)
	{
	}

	Softbody::Softbody(const Softbody& s) noexcept
	: CollisionObject((const CollisionObject&) s), points(s.points), springs(s.springs),
	width(s.width), height(s.height)
	{
	}

	Softbody::Softbody(Softbody && s) noexcept
	: CollisionObject((CollisionObject &&) s), points(s.points), springs(s.springs),
	width(s.width), height(s.height)
	{
	}

	Softbody::Softbody(const Transform& t,unsigned width, unsigned height) noexcept
	: CollisionObject(t, BoxCollider(), false), width(width), height(height)
	{
		geometry::Vector vec(0, height);
		for (unsigned i = 0; i < height; i++)
		{
			std::vector<MassPoint> arr;
			for (unsigned j = 0; j < width; j++)
			{
				MassPoint m;
				m.position = vec;
				m.mass = 1;
				arr.push_back(m);
				vec.x += 1;
			}
			points.push_back(arr);
			vec.x = 0;
			vec.y -= 1;
		}
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				Spring s;
				s.stiffness = 0.5;
				s.restingLength = 1;
				s.dampingFactor = 0.5;
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
				s.restingLength = sqrt(2);
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
				m.colliders.push_back(&(*_colliders.end()));
			}
		}
		SetCollider(m);
	}

	void Softbody::ApplyForce(const geometry::Vector& force, const geometry::Vector& point) noexcept
	{
		if (point == geometry::Vector::Infinity)
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
					if (m.position == point)
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

	/*void Softbody::FixCollapsing() noexcept
	{
		for (Spring& s: springs)
		{
			f64 length = geometry::Distance(s.a->position, s.b->position);
			if (!length)
				continue;
			if (length < 1)
			{
				
			}
		}
	}*/

	void Softbody::UpdateCollider() noexcept
	{
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
				m.colliders.push_back(&(*_colliders.end()));
			}
		}
		SetCollider(m);	
	}
}