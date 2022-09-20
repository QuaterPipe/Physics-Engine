#include "../../include/physics/Engine/DynamicsWorld.hpp"
#include <iostream>
#define MAXXVEL 100000
#define MAXYVEL 100000

namespace physics
{
	DynamicsWorld::DynamicsWorld() noexcept
	{
		_solvers.push_back(new PhysicsSolver());
	}

	void DynamicsWorld::AddObject(const CollisionObject& o) noexcept
	{
		_objects.push_back(o);
		_allObjects.push_back(&_objects[_objects.size() - 1]);
	}

	void DynamicsWorld::AddSolver(Solver* s) noexcept
	{
		_solvers.push_back(s);
	}

	void DynamicsWorld::AddRigidbody(const Rigidbody& rb) noexcept
	{
		_rigidbodies.push_back(rb);
	}

	void DynamicsWorld::AddSoftbody(const Softbody& sb) noexcept
	{
		Softbody copy(sb);
		if (copy.usesGravity)
			copy.gravity = _gravity;
		else
			copy.gravity.Set(0, 0);
		_softbodies.push_back(copy);
		_allObjects.push_back(&_softbodies[_softbodies.size() - 1]);
	}

	void DynamicsWorld::ApplyGravity(f64 dt) noexcept
	{
		for (auto& rb: _rigidbodies)
		{
			if (!rb.usesGravity)
				continue;
			rb.ApplyForce(dt, rb.gravity);
		}
		for (auto& sb: _softbodies)
		{
			if (!sb.usesGravity)
				continue;
			sb.ApplyForce(dt, sb.gravity);
		}
	}
	
	const std::vector<CollisionObject*>& DynamicsWorld::GetAllObjects() const noexcept
	{
		return _allObjects;
	}

	const std::vector<Rigidbody>& DynamicsWorld::GetRigidbodies() const noexcept
	{
		return _rigidbodies;
	}

	const std::vector<Softbody>& DynamicsWorld::GetSofftbodies() const noexcept
	{
		return _softbodies;
	}

	void DynamicsWorld::ResolveCollisions(f64 dt) noexcept
	{
		_collisions.clear();
		for (auto& a: _rigidbodies)
		{
			for (auto& b: _rigidbodies)
			{
				if (a == b)
					break;
				if (a.GetCollider().BoundingBox(a.transform).Overlaps(b.GetCollider().BoundingBox(b.transform)))
				{
					CollisionPoints points = a.GetCollider().TestCollision(
						a.transform, &b.GetCollider(), b.transform);
					if (points.hasCollision)
					{
						Collision c;
						c.a = &a;
						c.b = &b;
						c.points = points;
						_collisions.push_back(c);
					}
				}
			}
		}
		for (auto& a: _softbodies)
		{
			for (auto& b: _softbodies)
			{
				if (a == b)
					break;
				if (a.GetCollider().BoundingBox(a.transform).Overlaps(b.GetCollider().BoundingBox(b.transform)))
				{
					CollisionPoints points = a.GetCollider().TestCollision(
						a.transform, &b.GetCollider(), b.transform);
					if (points.hasCollision)
					{
						Collision c;
						c.a = &a;
						c.b = &b;
						c.points = points;
						_collisions.push_back(c);
					}
				}
			}
		}
	}

	void DynamicsWorld::RemoveObject(const CollisionObject& o) noexcept
	{
		for (auto p = _objects.begin(); p < _objects.end(); p++)	
		{
			if (*p == o)
			{
				_objects.erase(p);
				return;
			}
		}
		for (auto p = _allObjects.begin(); p < _allObjects.end(); p++)
		{
			if (**p == o)
			{
				_allObjects.erase(p);
				return;
			}
		}
	}

	void DynamicsWorld::RemoveSolver(Solver* s) noexcept
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
	
	void DynamicsWorld::RemoveRigidbody(const Rigidbody& rb) noexcept
	{
		for (auto p = _rigidbodies.begin(); p < _rigidbodies.end(); p++)
		{
			if (*p == rb)
			{
				_rigidbodies.erase(p);
				break;
			}
		}
		for (auto p = _allObjects.begin(); p < _allObjects.end(); p++)
		{
			if (**p == rb)
			{
				_allObjects.erase(p);
				break;
			}
		}
	}

	void DynamicsWorld::RemoveSoftbody(const Softbody& sb) noexcept
	{
		for (auto p = _softbodies.begin(); p < _softbodies.end(); p++)
		{
			if (*p == sb)
			{
				_softbodies.erase(p);
				break;
			}
		}
		for (auto p = _allObjects.begin(); p < _allObjects.end(); p++)
		{
			if (**p == sb)
			{
				_allObjects.erase(p);
				break;
			}
		}
	}

	void DynamicsWorld::SetCollisionCallBack(const std::function<void(Collision&, f64)>& callback, f64 dt) noexcept
	{
		_onCollision = callback;
	}

	void DynamicsWorld::SendCollisionCallBacks(std::vector<Collision>& collisions, f64 dt) noexcept
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

	void DynamicsWorld::MoveObjects(f64 dt) noexcept
	{
		// for (auto& rb: _rigidbodies)
		// 	rb.force += rb.gravity;//rb.IntegrateForces(dt);
		// for (auto& sb: _softbodies)
		// 	sb.IntegrateForces(dt);
	}

	void PositionalCorrectionSolve(Collision& c, f64 dt)
	{
		if (!c.a->IsDynamic() || !c.b->IsDynamic()) return;
		return;
		Rigidbody* a = (Rigidbody*) c.a;
		Rigidbody* b = (Rigidbody*) c.b;
		f64 percentage = 0.8;
		f64 slop = 0.05;
		geo::Vector2 correction = (std::max(c.points.depth - slop, 0.0) / (a->GetInvMass() + b->GetInvMass())) * percentage * c.points.normal;
		geo::Vector2 aPos = a->position;
		geo::Vector2 bPos = b->position;
		aPos -= a->GetInvMass() * correction;
		bPos += b->GetInvMass() * correction;
		
		if (!a->isKinematic && !a->isStatic)
			a->position = aPos;
		if (!b->isKinematic && !a->isStatic)
			b->position = bPos;
	}

	void DynamicsWorld::Update(f64 dt) noexcept
	{
		ResolveCollisions(dt);
		//MoveObjects(dt);
		for (auto& rb: _rigidbodies)
			rb.IntegrateForces(dt);
		SendCollisionCallBacks(_collisions, dt);
		for (auto solver: _solvers)
			solver->Solve(_collisions, dt);
		UpdateSoftbodies(dt);
		for (auto& rb: _rigidbodies)
			rb.IntegrateVelocity(dt);
		for (auto& sb: _softbodies)
			sb.IntegrateVelocity(dt);
		//for (auto& c: _collisions)
		//	PositionalCorrectionSolve(c, dt);
		for (auto& rb: _rigidbodies)
		{
			rb.force.Set(0, 0);
			rb.angularForce = 0;
		}
	}

	void DynamicsWorld::UpdateSoftbodies(f64 dt) noexcept
	{
		for (auto& sb: _softbodies)
		{
			sb.ApplySpringForces(dt);
		}
	}

}