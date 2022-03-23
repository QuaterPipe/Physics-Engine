#pragma once
#include <array>
#include <atomic>
#include <fstream>
#include <thread>
#include <mutex>
#include "../Collision/Collision.hpp"
#include "Entity.hpp"
#include "Display.hpp"
#include "World.hpp"

namespace physics
{
	class Scene;

	class Scene
	{
		private:
			class Smoother final
			{
				private:
					f64 _accumulator = 0;
					Scene* _s = NULL;
				public:
					Smoother() noexcept;
					Smoother(Scene& s) noexcept;
					void Update(f64 dt) noexcept;
					void PhysicsUpdate() noexcept;
			};
			struct Counter final
			{
				unsigned int loops;
				f64 total;
				std::vector<f64> loopsPerSecond;
			};
			Counter _fpsCounter;
			Counter _physicsUpdateCounter;
			Smoother _smoother;
			DynamicsWorld _world;
			std::atomic<std::vector<std::unique_ptr<Entity>>*> _entities;
			std::vector<sf::Texture> _textures;
			geometry::Vector _gravity = geometry::Vector(0, -9.81);
			Display* _display = NULL;
			std::atomic_ushort _physicsUpdateFrequency;
			std::atomic_bool _physicsIsActive;
			std::atomic_bool _ended;
			void _PhysicsLoop() noexcept;
			std::thread _physicsThread;
		public:
			Scene(const geometry::Vector& gravity, unsigned short physicsUpdateHz, unsigned windowWidth, unsigned windowHeight, std::string windowTitle="") noexcept;
			~Scene() noexcept;
			void AddEntity(Entity& e) noexcept;
			void AddTexture(sf::Texture t) noexcept;
			Display* GetDisplay() const noexcept;
			Entity& GetEntity(const size_t& index) noexcept;
			const std::vector<std::unique_ptr<Entity>>& GetEntities() const noexcept;
			unsigned short GetPhysicsUpdateFrequency() const noexcept;
			sf::Texture& GetTexture(const size_t& index) noexcept;
			const std::vector<sf::Texture>& GetTextures() const noexcept;
			void RemoveEntity(Entity& e) noexcept;
			void StartPhysics() noexcept;
			void StopPhysics() noexcept;
			void SetGravity(const geometry::Vector& g) noexcept;
			void SetPhysicsUpdateFrequency(const unsigned short& ms) noexcept;
			void Update(f64 dt) noexcept;
	};
}