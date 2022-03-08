#pragma once
#include "Collision.hpp"
#include "Rigidbody.hpp"

namespace physics
{
	class Solver
	{
		public:
			virtual void Solve(std::vector<Collision>& collisions, f64 dt) noexcept = 0;
	};

	class FrictionSolver : public Solver
	{
		public:
			void Solve(std::vector<Collision>& collisions, f64 dt) noexcept override;
	};

	class ImpulseSolver : public Solver
	{
		public:
			void Solve(std::vector<Collision>& collisions, f64 dt) noexcept override;
	};

	class PositionalCorrectionSolver : public Solver
	{
		public:
			void Solve(std::vector<Collision>& collisions, f64 dt) noexcept override;
	};
	
	class DynamicsWorld;
	class CollisionWorld;
	class CollisionWorld
	{
		protected:
			std::vector<CollisionObject*> _objects;
			std::vector<Solver*> _solvers;
			std::function<void(Collision&, f64)> _onCollision;
			geometry::Vector _gravity = geometry::Vector(0, -9.81);
		public:
			void AddObject(CollisionObject* o) noexcept;
			void AddSolver(Solver* s) noexcept;
			void ResolveCollisions(f64 dt) noexcept;
			void RemoveObject(CollisionObject* o) noexcept;
			void RemoveSolver(Solver* s) noexcept;
			void SetCollisionCallBack(std::function<void(Collision&, f64)>& callback, f64 dt) noexcept;
			void SendCollisionCallBacks(std::vector<Collision>& collisions, f64 dt) noexcept;
	};

	class DynamicsWorld : public CollisionWorld
	{
		private:
			geometry::Vector _gravity = geometry::Vector(0, -9.81);
		public:
			DynamicsWorld() noexcept;
			void AddRigidbody(Rigidbody* rb) noexcept;
			void ApplyGravity(f64 dt) noexcept;
			void MoveObjects(f64 dt) noexcept;
			void Update(f64 dt) noexcept;
	};
}