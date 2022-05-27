#pragma once
#include "../Collision/Collision.hpp"
#include "../Dynamics/Rigidbody.hpp"
#include "../Dynamics/Softbody.hpp"

namespace physics
{
	class Solver
	{
		public:
			virtual void Solve(std::vector<Collision>& collisions, f64 dt) noexcept = 0;
	};

	class PhysicsSolver : public Solver
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
			geo::Vector _gravity = geo::Vector(0, -9.81);
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
			geo::Vector _gravity = geo::Vector(0, -9.81);
		public:
			DynamicsWorld() noexcept;
			void AddRigidbody(Rigidbody* rb) noexcept;
			void AddSoftbody(Softbody *sb) noexcept;
			void ApplyGravity(f64 dt) noexcept;
			void MoveObjects(f64 dt) noexcept;
			void Update(f64 dt) noexcept;
			void UpdateSoftbodies(f64 dt) noexcept;
	};
}