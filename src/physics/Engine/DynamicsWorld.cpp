#include "physics/Engine/DynamicsWorld.hpp"
#include <iostream>
#define MAXXVEL 100000
#define MAXYVEL 100000

namespace physics
{
	DynamicsWorld::DynamicsWorld(BoxCollider area) noexcept
		: quadtree(0, 8, 10, BoxCollider(area), &_objects)
	{
		_solvers.push_back(new PhysicsSolver());
		_solvers.push_back(new PositionalCorrectionSolver());
	}

	void DynamicsWorld::AddConstraint(Constraint* c) noexcept
	{
		if (!c)
			return;
		_constraints.push_back(c);
	}

	void DynamicsWorld::AddDynamicbody(Dynamicbody* dynamicbody) noexcept
	{
		if (!dynamicbody)
			return;
		_dynamicbodies.push_back(dynamicbody);
		_objects.push_back(dynamicbody);
	}

	void DynamicsWorld::AddObject(CollisionObject* o) noexcept
	{
		if (!o)
			return;
		_objects.push_back(o);
	}

	void DynamicsWorld::AddSolver(Solver* s) noexcept
	{
		if (!s)
			return;
		_solvers.push_back(s);
	}

	void DynamicsWorld::ApplyGravity(f64 dt) noexcept
	{
		for (auto& db: _dynamicbodies)
		{
			if (!db->usesGravity)
				continue;
			db->velocity += db->gravity * dt;
		}
	}

	void DynamicsWorld::CheckCollisions(f64 dt) noexcept
	{
		_collisions.clear();
		std::vector<std::vector<int>> container;
		quadtree.Get(container);
		for (size_t k = 0; k < container.size(); k++)
		{
			for (size_t i = 0; i < container[k].size(); i++)
			{
				for (size_t j = i + 1; j < container[k].size(); j++)
				{
					CollisionObject* tmpA = _objects[container[k][i]];
					Dynamicbody* a = nullptr;
					if (tmpA->IsDynamic())
						a = dynamic_cast<Dynamicbody*>(tmpA);
					else
						continue;
					Dynamicbody* b = nullptr;
					CollisionObject* tmpB = _objects[container[k][j]];
					if (tmpB->IsDynamic())
						b = dynamic_cast<Dynamicbody*>(tmpB);
					else
						continue;
					if (!a || !b)
						continue;
					if ((!a->isActive || !b->isActive))
						continue;
					continue;
					if (a->GetCollider().BoundingBox(a->transform).Overlaps(b->GetCollider().BoundingBox(b->transform)))
					{
						Manifold points = a->GetCollider().TestCollision(
							a->transform, &b->GetCollider(), b->transform);
						if (points.hasCollision)
						{
							CollisionManifold c;
							c.a = a;
							c.b = b;
							c.points = points;
							_collisions.push_back(c);
						}
					}
				}
			}
		}
	}

	const std::vector<CollisionObject*>& DynamicsWorld::GetAllObjects() const noexcept
	{
		return _objects;
	}

	void DynamicsWorld::RemoveConstraint(Constraint* c) noexcept
	{
		if (!c)
			return;
		for (size_t i = 0; i < _constraints.size(); i++)
		{
			if (_constraints[i] == c)
			{
				_constraints.erase(_constraints.begin() + i);
				return;
			}

		}
	}

	void DynamicsWorld::RemoveDynamicbody(Dynamicbody* dynamicbody) noexcept
	{
		if (!dynamicbody)
			return;
		for (auto p = _dynamicbodies.begin(); p != _dynamicbodies.end(); p++)
		{
			if (*p == dynamicbody)
			{
				_dynamicbodies.erase(p);
				break;
			}
		}
		for (auto p = _objects.begin(); p != _objects.end(); p++)
		{
			if (*p == dynamicbody)
			{
				_objects.erase(p);
				return;
			}
		}
	}

	void DynamicsWorld::RemoveObject(CollisionObject* o) noexcept
	{
		if (!o)
			return;
		for (auto p = _objects.begin(); p < _objects.end(); p++)	
		{
			if (*p == o)
			{
				_objects.erase(p);
				return;
			}
		}
	}

	void DynamicsWorld::RemoveSolver(Solver* s) noexcept
	{
		if (!s)
			return;
		for (auto p = _solvers.begin(); p < _solvers.end(); p++)
		{
			if (*p == s)
			{
				_solvers.erase(p);
				return;
			}
		}
	}
	
	void DynamicsWorld::SetCollisionCallBack(const std::function<void(CollisionManifold&, f64)>& callback, f64 dt) noexcept
	{
		_onCollision = callback;
	}

	void DynamicsWorld::SendCollisionCallBacks(f64 dt) noexcept
	{
		for (CollisionManifold& c: _collisions)
		{
			if (_onCollision) _onCollision(c, dt);
			if (c.a->onCollision) c.a->onCollision(c, dt);
			if (c.b->onCollision) c.b->onCollision(c, dt);
		}
	}
	
	void DynamicsWorld::Update(f64 dt) noexcept
	{
		
		ApplyGravity(dt);
		for (int i = 0; i < 4; i++)
			for (auto& db : _dynamicbodies)
				db->Update(dt, i);
		quadtree.Update();
		CheckCollisions(dt);
		SendCollisionCallBacks(dt);

		for (Constraint* c : _constraints)
		{
			c->WarmStart();
			c->Reset();
		}
		f64 constraintDt = dt / constraintIterationCount;
		for (int j = 0; j < constraintIterationCount; j++)
		{
			for (Constraint* c : _constraints)
				c->UpdateConstraint(constraintDt);
		}
		_solvers[0]->Solve(_collisions, dt);
		_solvers[1]->Solve(_collisions, dt);
	}
}