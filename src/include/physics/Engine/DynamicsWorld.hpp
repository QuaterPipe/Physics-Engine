#pragma once
#include "../Collision/Collision.hpp"
#include "../Dynamics/Rigidbody.hpp"
#include "../Dynamics/Softbody.hpp"
#include "../Dynamics/VelocityConstraint.hpp"
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
	
	class DynamicsWorld
	{
		protected:
			std::function<void(Collision&, f64)> _onCollision;
			geo::Vector2 _gravity = geo::Vector2(0, -9.81);
			std::vector<CollisionObject*> _allObjects;
			std::vector<Collision> _collisions;
			std::vector<CollisionObject> _objects;
			std::vector<Rigidbody> _rigidbodies;
			std::vector<Softbody> _softbodies;
			std::vector<Solver*> _solvers;
			std::vector<Constraint*> _constraints;
		public:
			DynamicsWorld() noexcept;
			void AddObject(const CollisionObject& o) noexcept;
			void AddRigidbody(const Rigidbody& rb) noexcept;
			void AddSoftbody(const Softbody& sb) noexcept;
			void AddSolver(Solver* s) noexcept;
			void ApplyGravity(f64 dt) noexcept;
			const std::vector<CollisionObject*>& GetAllObjects() const noexcept;
			const std::vector<Rigidbody>& GetRigidbodies() const noexcept;
			const std::vector<Softbody>& GetSofftbodies() const noexcept;
			void MoveObjects(f64 dt) noexcept;
			void ResolveCollisions(f64 dt) noexcept;
			void RemoveObject(const CollisionObject& o) noexcept;
			void RemoveRigidbody(const Rigidbody& rb) noexcept;
			void RemoveSolver(Solver* s) noexcept;
			void RemoveSoftbody(const Softbody& sb) noexcept;
			void SendCollisionCallBacks(std::vector<Collision>& collisions, f64 dt) noexcept;
			void SetCollisionCallBack(const std::function<void(Collision&, f64)>& callback, f64 dt) noexcept;
			void Update(f64 dt) noexcept;
			void UpdateSoftbodies(f64 dt) noexcept;
	};
}