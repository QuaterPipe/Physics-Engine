#include "../include/physics/World.hpp"
#include "../include/physics/OstreamOverloads.hpp"
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
			geometry::Vector aPos = a->GetPosition();
			geometry::Vector bPos = b->GetPosition();
			aPos -= a->GetInvMass() * correction;
			bPos += b->GetInvMass() * correction;
			if (!a->IsKinematic())
				a->SetPosition(aPos);
			if (!b->IsKinematic())
				b->SetPosition(bPos);
		}
	}

	template <typename T>
	T getMin(std::vector<T>& v)	
	{
		bool reachedVal = false;
		T minVal;
		for (T item: v)
		{
			if (!reachedVal)
				minVal = item; reachedVal = true; continue;
			if (minVal > item)
				minVal = item;
		}
		return minVal;
	}

	template <typename T>
	T getMax(std::vector<T>& v)	
	{
		bool reachedVal = false;
		T maxVal;
		for (T item: v)
		{
			if (!reachedVal)
			{
				maxVal = item;
				reachedVal = true;
				continue;
			}
			if (maxVal < item)
				maxVal = item;
		}
		return maxVal;
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
				Transform trans = a->GetTransform();
				geometry::Vector min = cldr.Min() + a->GetPosition();
				geometry::Vector max = cldr.Max() + a->GetPosition();
				BoundingBoxA.x = min.x;
				BoundingBoxA.y = min.y;
				BoundingBoxA.width = max.x - min.x;
				BoundingBoxA.height = max.y - min.y;
				// b collider to bounding box
				cldr = b->GetCollider();
				trans = b->GetTransform();
				min = cldr.Min() + b->GetPosition();
				max = cldr.Max() + b->GetPosition();
				BoundingBoxB.x = min.x;
				BoundingBoxB.y = min.y;
				BoundingBoxB.width = max.x - min.x;
				BoundingBoxB.height = max.y - min.y;
				if (SquareOverLaps(BoundingBoxA, BoundingBoxB))
				{
					CollisionPoints points = a->GetCollider().TestCollision(
						a->GetTransform(), &b->GetCollider(), b->GetTransform()
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
			const auto& a = c.a->GetOnCollisionFunction();
			const auto& b = c.b->GetOnCollisionFunction();
			if (a) a(c, dt);
			if (b) b(c, dt);
		}
	}

	void DynamicsWorld::AddRigidbody(Rigidbody* r) noexcept
	{
		if (!r) {return;}
		if (r->UsesGravity())
			r->SetGravity(_gravity);
		else
			r->SetGravity(geometry::Vector(0, 0));
		_objects.push_back(r);
	}

	void DynamicsWorld::ApplyGravity(f64 dt) noexcept
	{
		for (auto& obj: _objects)
		{
			if (!obj->IsDynamic()) continue;
			Rigidbody* rigidbody = (Rigidbody*) obj;
			if (!rigidbody->UsesGravity()) continue;
			if (rigidbody->IsKinematic()) continue;
			rigidbody->ApplyForce(rigidbody->GetGravity());
		}
	}

	void DynamicsWorld::MoveObjects(f64 dt) noexcept
	{
		for (auto& obj: _objects)
		{
			if (!obj->IsDynamic()) continue;
			Rigidbody* rigidbody = (Rigidbody*)obj;
			if (rigidbody->IsKinematic()) continue;
			geometry::Vector newPosition;
			if (rigidbody->GetVelocity().x > MAXXVEL)
			{
				rigidbody->SetVelocity(geometry::Vector(MAXXVEL, rigidbody->GetVelocity().y));
			}
			if (rigidbody->GetVelocity().y > MAXYVEL)
			{
				rigidbody->SetVelocity(geometry::Vector(rigidbody->GetVelocity().x, MAXYVEL));
			}
			newPosition = rigidbody->GetVelocity() * rigidbody->GetInvMass() + rigidbody->GetPosition();
			Transform t = rigidbody->GetTransform();
			t.rotation.Set(acos(t.rotation.a) + rigidbody->GetAngularVelocity());
			t.position.Set(newPosition.x, newPosition.y);
			rigidbody->SetTransform(t);
			//rigidbody->Update(dt);
		}
	}

	void DynamicsWorld::Update(f64 dt) noexcept
	{
		ApplyGravity(dt);
		ResolveCollisions(dt);
		MoveObjects(dt);
	}

	DynamicsWorld::DynamicsWorld() noexcept
	{
		_solvers.push_back(new PhysicsSolver());
		//_solvers.push_back(new PositionalCorrectionSolver());
	}
}