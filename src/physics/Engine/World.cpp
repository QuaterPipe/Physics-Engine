#include "../../include/physics/Engine/World.hpp"
#include "../../include/physics/Tools/OstreamOverloads.hpp"
#include <iostream>
#define MAXXVEL 100000
#define MAXYVEL 100000

namespace physics
{
	void PositionalCorrectionSolver::Solve(std::vector<Collision>& collisions, f64 dt) noexcept
	{
		for (Collision& c: collisions)
		{
			if (!c.a->IsDynamic() || !c.b->IsDynamic()) continue;
			Rigidbody* a = (Rigidbody*) c.a;
			Rigidbody* b = (Rigidbody*) c.b;
			double percentage = 0.2;
			double slop = 0.01;
			geometry::Vector correction = std::max(c.points.depth - slop, 0.0) / (a->GetInvMass() + b->GetInvMass()) * percentage * c.points.normal;
			geometry::Vector aPos = a->position;
			geometry::Vector bPos = b->position;
			aPos -= a->GetInvMass() * correction;
			bPos += b->GetInvMass() * correction;
			if (!a->isKinematic && !a->isStatic)
				a->position = aPos;
			if (!b->isKinematic && !a->isStatic)
				b->position = bPos;
		}
	}

	struct Square
	{
		f64 x = 0;
		f64 y = 0;
		f64 width = 0;
		f64 height = 0;
	};

	bool SquareOverLaps(const Square& a, const Square& b)
	{
		auto numInRange = [&] (double value, double minVal, double maxVal){
			return (value >= minVal) && (value <= maxVal);
		};
		bool xOverlaps = numInRange(a.x, b.x, b.x + b.width) ||
			numInRange(b.x, a.x, a.x + a.width);
		bool yOverlaps = numInRange(a.y, b.y, b.y + b.height) ||
			numInRange(b.y, a.y, a.y + a.height);
		return xOverlaps && yOverlaps;
	}

	void CollisionWorld::AddObject(CollisionObject* o) noexcept
	{
		_objects.push_back(o);
	}

	void CollisionWorld::AddSolver(Solver* s) noexcept
	{
		_solvers.push_back(s);
	}

	void CollisionWorld::ResolveCollisions(f64 dt) noexcept
	{
		std::vector<Collision> collisions;
		for (auto& a: _objects)
		{
			for (auto& b: _objects)
			{
				if (a == b) break;
				Square BoundingBoxA;
				Square BoundingBoxB;
				// a collider to bounding box
				Collider& cldr = a->GetCollider();
				Transform trans = a->transform;
				geometry::Vector min = cldr.Min() + a->position;
				geometry::Vector max = cldr.Max() + a->position;
				BoundingBoxA.x = min.x;
				BoundingBoxA.y = min.y;
				BoundingBoxA.width = max.x - min.x;
				BoundingBoxA.height = max.y - min.y;
				// b collider to bounding box
				cldr = b->GetCollider();
				trans = b->transform;
				min = cldr.Min() + b->position;
				max = cldr.Max() + b->position;
				BoundingBoxB.x = min.x;
				BoundingBoxB.y = min.y;
				BoundingBoxB.width = max.x - min.x;
				BoundingBoxB.height = max.y - min.y;
				if (SquareOverLaps(BoundingBoxA, BoundingBoxB))
				{
					CollisionPoints points = a->GetCollider().TestCollision(
						a->transform, &b->GetCollider(), b->transform
					);
					if (points.hasCollision)
					{
						Collision c;
						c.a = a;
						c.b = b;
						c.points = points;
						collisions.push_back(c);
					}
				}
			}
		}
		for (auto solver: _solvers)
		{
			solver->Solve(collisions, dt);
		}
	}

	void CollisionWorld::RemoveObject(CollisionObject* o) noexcept
	{
		for (auto p = _objects.begin(); p < _objects.end(); p++)	
		{
			if (*p == o)
			{
				_objects.erase(p);
				return;
			}
		}
	}

	void CollisionWorld::RemoveSolver(Solver* s) noexcept
	{
		for (auto p = _solvers.begin(); p < _solvers.end(); p++)
		{
			if (*p == s)
			{
				_solvers.erase(p);
				return;
			}
		}
	}

	void CollisionWorld::SetCollisionCallBack(std::function<void(Collision&, f64)>& callback, f64 dt) noexcept
	{
		_onCollision = callback;
	}

	void CollisionWorld::SendCollisionCallBacks(std::vector<Collision>& collisions, f64 dt) noexcept
	{
		for (Collision& c: collisions)
		{
			_onCollision(c, dt);
			const auto& a = c.a->onCollision;
			const auto& b = c.b->onCollision;
			if (a) a(c, dt);
			if (b) b(c, dt);
		}
	}

	void DynamicsWorld::AddRigidbody(Rigidbody* r) noexcept
	{
		if (!r) {return;}
		if (r->usesGravity)
			r->gravity = _gravity;
		else
			r->gravity.Set(0, 0);
		_objects.push_back(r);
	}

	void DynamicsWorld::AddSoftbody(Softbody* sb) noexcept
	{
		if (!sb) return;
		_objects.push_back(sb);
	}

	void DynamicsWorld::ApplyGravity(f64 dt) noexcept
	{
		for (auto& obj: _objects)
		{
			if (!obj->IsDynamic()) continue;
			Rigidbody* rigidbody = dynamic_cast<Rigidbody*>(obj);
			if (rigidbody)
			{
				if (!rigidbody->usesGravity) continue;
				if (rigidbody->isKinematic || rigidbody->isStatic) continue;
				rigidbody->ApplyForce(rigidbody->gravity);
				continue;
			}
			Softbody* softbody = dynamic_cast<Softbody*>(obj);
			if (softbody && softbody->usesGravity && !softbody->isStatic)
			{
				softbody->ApplyForce(softbody->gravity);
			}
		}
	}

	void DynamicsWorld::MoveObjects(f64 dt) noexcept
	{
		for (auto& obj: _objects)
		{
			if (!obj->IsDynamic()) continue;
			Rigidbody* rigidbody = dynamic_cast<Rigidbody*>(obj);
			if (rigidbody && !rigidbody->isKinematic && !rigidbody->isStatic)
			{
				rigidbody->Update(dt);
				continue;
			}
			Softbody* softbody = dynamic_cast<Softbody*>(obj);
			if (softbody && !softbody->isStatic)
			{
				softbody->Update(dt);
			}
		}
	}

	void DynamicsWorld::Update(f64 dt) noexcept
	{
		ApplyGravity(dt);
		UpdateSoftbodies(dt);
		ResolveCollisions(dt);
		MoveObjects(dt);
	}

	void DynamicsWorld::UpdateSoftbodies(f64 dt) noexcept
	{
		for (auto& obj: _objects)
		{
			if (!obj->IsDynamic()) continue;
			Softbody* softbody = dynamic_cast<Softbody*>(obj);
			if (softbody)
				softbody->ApplySpringForces();
		}
	}

	DynamicsWorld::DynamicsWorld() noexcept
	{
		_solvers.push_back(new PhysicsSolver());
		_solvers.push_back(new PositionalCorrectionSolver());
	}
}