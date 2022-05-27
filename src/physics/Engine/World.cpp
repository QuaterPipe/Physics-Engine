#include "../../include/physics/Engine/World.hpp"
#include "../../include/physics/Tools/OstreamOverloads.hpp"
#include <iostream>
#define MAXXVEL 100000
#define MAXYVEL 100000

namespace physics
{

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
				if (a->GetCollider().BoundingBox(a->transform).Overlaps(b->GetCollider().BoundingBox(b->transform)))
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
			solver->Solve(collisions, dt);
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
		if (!r) return;
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
			if (rigidbody)
			{
				rigidbody->Update(dt);
			}
			Softbody* softbody = dynamic_cast<Softbody*>(obj);
			if (softbody)
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
		//_solvers.push_back(new PositionalCorrectionSolver());
	}
}