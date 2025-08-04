#pragma once
#include "ConstraintSolver.hpp"
#include "physics/Collision/Collision.hpp"
#include "physics/Dynamics/Rigidbody.hpp"
#include "physics/Dynamics/Softbody.hpp"
#include "Quadtree.hpp"
#include <algorithm>

namespace physics
{
	class Solver
	{
		public:
			virtual void Solve(std::vector<CollisionManifold>& collisions, f64 dt) noexcept = 0;
			Vector2 gravity = Vector2(0, 9.81);
	};

	class PhysicsSolver : public Solver
	{
		public:
			void Solve(std::vector<CollisionManifold>& collisions, f64 dt) noexcept override;
	};

	class PositionalCorrectionSolver : public Solver
	{
		public:
			void Solve(std::vector<CollisionManifold>& collision, f64 dt) noexcept override;
	};
	
	class DynamicsWorld
	{
		protected:
			std::function<void(CollisionManifold&, f64)> _onCollision;
			Vector2 _gravity = Vector2(0, -9.81);
			std::vector<Constraint*> _constraints;
			std::vector<CollisionObject*> _objects;
			std::vector<Dynamicbody*> _dynamicbodies;
			std::vector<CollisionManifold> _collisions;
			std::vector<Solver*> _solvers;
		public:
			size_t constraintIterationCount = 16;
			Quadtree quadtree;
			DynamicsWorld(BoxCollider area) noexcept;
			void AddConstraint(Constraint* c) noexcept;
			void AddDynamicbody(Dynamicbody* dynamicbody) noexcept;
			void AddObject(CollisionObject* o) noexcept;
			void AddSolver(Solver* s) noexcept;
			void ApplyGravity(f64 dt) noexcept;
			void CheckCollisions(f64 dt) noexcept;
			const std::vector<CollisionObject*>& GetAllObjects() const noexcept;
			void RemoveConstraint(Constraint* c) noexcept;
			void RemoveDynamicbody(Dynamicbody* dynamicbody) noexcept;
			void RemoveObject(CollisionObject* o) noexcept;
			void RemoveSolver(Solver* s) noexcept;
			void ResolveConstraints(Constraint* c) noexcept;
			void SendCollisionCallBacks(f64 dt) noexcept;
			void SetCollisionCallBack(const std::function<void(CollisionManifold&, f64)>& callback, f64 dt) noexcept;
			void Update(f64 dt) noexcept;
	};
}