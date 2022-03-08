#pragma once
#include <array>
#include <fstream>
#include "Collision.hpp"
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
			std::vector<std::unique_ptr<Entity>> _entities;
			std::array<sf::Texture> _textures;
			geometry::Vector _gravity = geometry::Vector(0, -9.81);
		public:
			Scene(const geometry::Vector& gravity) noexcept;
			~Scene() noexcept;
			void AddEntity(Entity& e) noexcept;
			void AddTexture(sf::Texture t) noexcept;
			Entity& GetEntity(const size_t& index) noexcept;
			const std::vector<std::unique_ptr<Entity>>& GetEntities() const noexcept;
			sf::Texture& GetTexture(const size_t& index) noexcept;
			const std::array<sf::Texture>& GetTextures() const noexcept;
			void RemoveEntity(Entity& e) noexcept;
			void SetGravity(const geometry::Vector& g) noexcept;
			void Update(f64 dt) noexcept;
			unsigned short physicsUpdateFrequency = 60;
			Display* display = NULL;
	};
}