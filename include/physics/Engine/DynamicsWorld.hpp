#pragma once
#include "ConstraintSolver.hpp"
#include "physics/Collision/Collision.hpp"
#include "physics/Dynamics/Rigidbody.hpp"
#include "physics/Dynamics/Softbody.hpp"
#include "physics/Dynamics/VelocityConstraint.hpp"
#include <algorithm>

namespace physics
{
	class Solver
	{
		public:
			virtual void Solve(std::vector<Collision>& collisions, f64 dt) noexcept = 0;
			geo::Vector2 gravity = geo::Vector2(0, 9.81);
	};

	class PhysicsSolver : public Solver
	{
		public:
			void Solve(std::vector<Collision>& collisions, f64 dt) noexcept override;
	};

	class PositionalCorrectionSolver : public Solver
	{
		public:
			void Solve(std::vector<Collision>& collision, f64 dt) noexcept override;
	};
	
	class DynamicsWorld
	{
		protected:
			std::function<void(Collision&, f64)> _onCollision;
			geo::Vector2 _gravity = geo::Vector2(0, -9.81);
			std::vector<CollisionObject*> _objects;
			std::vector<Dynamicbody*> _dynamicbodies;
			std::vector<Collision> _collisions;
			std::vector<Solver*> _solvers;
		public:
			DynamicsWorld() noexcept;
			void AddObject(CollisionObject* o) noexcept;
			void AddDynamicbody(Dynamicbody* dynamicbody) noexcept;
			void AddSolver(Solver* s) noexcept;
			void ApplyGravity(f64 dt) noexcept;
			void CheckCollisions(f64 dt) noexcept;
			const std::vector<CollisionObject*>& GetAllObjects() const noexcept;
			void IntegrateObjects(f64 dt) noexcept;
			void RemoveObject(CollisionObject* o) noexcept;
			void RemoveDynamicbody(Dynamicbody* dynamicbody) noexcept;
			void RemoveSolver(Solver* s) noexcept;
			void ResolveConstraints(f64 dt) noexcept;
			void SendCollisionCallBacks(f64 dt) noexcept;
			void SetCollisionCallBack(const std::function<void(Collision&, f64)>& callback, f64 dt) noexcept;
			void Update(f64 dt) noexcept;
	};
}