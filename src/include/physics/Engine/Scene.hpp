#pragma once
#include <array>
#include <atomic>
#include <fstream>
#include <thread>
#include <mutex>
#include <queue>
#include "../Collision/Collision.hpp"
#include "Entity.hpp"
#include "../Events/EventListener.hpp"
#include "Display.hpp"
#include "World.hpp"

namespace physics
{
	class Scene;

	class Scene
	{
		private:
			struct Counter final
			{
				unsigned int loops;
				f64 total;
				std::vector<f64> loopsPerSecond;
			};
			Counter _fpsCounter;
			Counter _physicsUpdateCounter;
			DynamicsWorld _world;
			std::atomic<std::vector<std::unique_ptr<Entity>>*> _entities;
			geo::Vector _gravity = geo::Vector(0, -9.81);
			Display* _display = NULL;
			event::EventListener _listener;
			std::atomic_ushort _physicsUpdateFrequency;
			std::atomic_bool _physicsIsActive;
			std::atomic_bool _ended;
			void _PhysicsLoop() noexcept;
			bool _started;
			std::thread _physicsThread;
		public:
			std::vector<std::function<void(const event::Event&)>> eventCallbacks;
			std::queue<event::Event*> eventQueue;
			Scene(const geo::Vector& gravity, unsigned short physicsUpdateHz, unsigned windowWidth, unsigned windowHeight, std::string windowTitle="") noexcept;
			virtual ~Scene() noexcept;
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
			void SetGravity(const geo::Vector& g) noexcept;
			void SetPhysicsUpdateFrequency(const unsigned short& ms) noexcept;
			void Update(f64 dt) noexcept;
	};
}