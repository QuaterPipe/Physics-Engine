#include "physics/Engine/DynamicsWorld.hpp"
#include <iostream>
#define MAXXVEL 100000
#define MAXYVEL 100000

namespace physics
{
	DynamicsWorld::DynamicsWorld() noexcept
	{
		_solvers.push_back(new PhysicsSolver());
		_solvers.push_back(new PositionalCorrectionSolver());
	}

	void DynamicsWorld::AddObject(CollisionObject* o) noexcept
	{
		_objects.push_back(o);
	}

	void DynamicsWorld::AddSolver(Solver* s) noexcept
	{
		_solvers.push_back(s);
	}

	void DynamicsWorld::AddDynamicbody(Dynamicbody* dynamicbody) noexcept
	{
		_dynamicbodies.push_back(dynamicbody);
		_objects.push_back(dynamicbody);
	}

	void DynamicsWorld::ApplyGravity(f64 dt) noexcept
	{
		for (auto& db: _dynamicbodies)
		{
			if (!db->usesGravity)
				continue;
			db->ApplyForce(db->gravity * dt);
		}
	}

	void DynamicsWorld::CheckCollisions(f64 dt) noexcept
	{
		_collisions.clear();
		for (auto& a: _dynamicbodies)
		{
			for (auto& b: _dynamicbodies)
			{
				if (a == b)
					continue;
				if (a->GetCollider().BoundingBox(a->transform).Overlaps(b->GetCollider().BoundingBox(b->transform)))
				{
					CollisionPoints points = a->GetCollider().TestCollision(
						a->transform, &b->GetCollider(), b->transform);
					if (points.hasCollision)
					{
						Collision c;
						c.a = a;
						c.b = b;
						c.points = points;
						_collisions.push_back(c);
					}
				}
			}
		}
	}

	const std::vector<CollisionObject*>& DynamicsWorld::GetAllObjects() const noexcept
	{
		return _objects;
	}

	void DynamicsWorld::RemoveObject(CollisionObject* o) noexcept
	{
		for (auto p = _objects.begin(); p < _objects.end(); p++)	
		{
			if (*p == o)
			{
				_objects.erase(p);
				return;
			}
		}
		for (auto p = _dynamicbodies.begin(); p < _dynamicbodies.end(); p++)
		{
			if (*p == o)
			{
				_dynamicbodies.erase(p);
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
	
	void DynamicsWorld::SetCollisionCallBack(const std::function<void(Collision&, f64)>& callback, f64 dt) noexcept
	{
		_onCollision = callback;
	}

	void DynamicsWorld::SendCollisionCallBacks(std::vector<Collision>& collisions, f64 dt) noexcept
	{
		for (Collision& c: collisions)
		{
			if (_onCollision) _onCollision(c, dt);
			if (c.a->onCollision) c.a->onCollision(c, dt);
			if (c.b->onCollision) c.b->onCollision(c, dt);
		}
	}

	void DynamicsWorld::IntegrateObjects(f64 dt) noexcept
	{
		for (auto& db: _dynamicbodies)
			db->IntegrateVelocity(dt);
	}
	
	void DynamicsWorld::Update(f64 dt) noexcept
	{
		for (auto& db : _dynamicbodies)
			db->IntegrateForces(dt);
		_solvers[0]->Solve(_collisions, dt);
		IntegrateObjects(dt);
		_solvers[1]->Solve(_collisions, dt);
		for (auto& db : _dynamicbodies)
		{
			db->force.Set(0, 0);
			db->angularForce = 0;
		}
		CheckCollisions(dt);
		SendCollisionCallBacks(_collisions, dt);
	}
}