#include "../../include/physics/Engine/Scene.hpp"
#include "../../include/physics/Engine/Time.hpp"
#include <iostream>

namespace physics
{

	Scene::Scene(const geo::Vector& gravity, unsigned short physicsUpdateHz, unsigned windowWidth, unsigned windowHeight, std::string windowTitle) noexcept
	: _physicsUpdateFrequency(physicsUpdateHz), _gravity(gravity), _display(new Display(windowWidth, windowHeight, windowTitle))
	{
		_ended.store(false, std::memory_order_relaxed);
		_entities.store(new std::vector<std::unique_ptr<Entity>>(), std::memory_order_relaxed);
		// sfml displays y coordinates from top to bottom, so inverting the y value in view fixes this, displaying y coordinates from bottom to top.
		sf::View v = _display->GetWindow()->getView();
		v.setSize(windowWidth, -((signed int)windowHeight));
		_display->SetView(v);
		_physicsIsActive.store(false);
		_physicsThread = std::thread(&Scene::_PhysicsLoop, this);
		_started = false;
	}

	Scene::~Scene() noexcept
	{
		_ended.store(true, std::memory_order_relaxed);
		_physicsThread.join();
		delete _display;
	}

	const std::vector<std::unique_ptr<Entity>>& Scene::GetEntities() const noexcept
	{
		return *_entities.load(std::memory_order_relaxed);
	}

	void Scene::AddEntity(Entity& e) noexcept
	{
		Entity* ptr = &e;
		if (ptr->GetCollisionObject().IsDynamic())
		{
			if (dynamic_cast<Rigidbody*>(&ptr->GetCollisionObject()))
				_world.AddRigidbody(dynamic_cast<Rigidbody*>(&ptr->GetCollisionObject()));
			else if (dynamic_cast<Softbody*>(&ptr->GetCollisionObject()))
				_world.AddSoftbody(dynamic_cast<Softbody*>(&ptr->GetCollisionObject()));
		}
		else
		{
			_world.AddObject(&ptr->GetCollisionObject());
		}
		(*_entities.load(std::memory_order_relaxed)).emplace_back(ptr);
	}

	Display* Scene::GetDisplay() const noexcept
	{
		return _display;
	}

	Entity& Scene::GetEntity(const size_t& index) noexcept
	{
		return *((*_entities.load(std::memory_order_relaxed)).at(index));
	}

	unsigned short Scene::GetPhysicsUpdateFrequency() const noexcept
	{
		return _physicsUpdateFrequency;
	}

	void Scene::_PhysicsLoop() noexcept
	{
		Timer timer;
		unsigned short hz = _physicsUpdateFrequency.load(std::memory_order_relaxed);
		while (true)
		{
			timer.Start();
			_world.Update(1.f / (double)hz);
			_world.ResolveCollisions(1.f / (double)hz);
			for (auto& ptr: *_entities.load(std::memory_order_relaxed))
			{
				ptr->FixedUpdate();
				ptr->Update();
			}
			hz = _physicsUpdateFrequency.load(std::memory_order_relaxed);
			if (_ended.load(std::memory_order_relaxed))
				break;
			while (!_physicsIsActive.load(std::memory_order_relaxed))
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(1 / hz * 1000));
			}
			timer.Stop();
			if (timer.deltaTime >= (1.f / (double)hz * 1000.f)) // calculations took too long
				continue;
			else
			{
				double d = (1.f / (double)hz * 1000.f) - timer.deltaTime;
				std::this_thread::sleep_for(std::chrono::microseconds((int)(d * 1000)));
			}
			timer.Reset();
		}
	}

	void Scene::RemoveEntity(Entity& e) noexcept
	{
		size_t ind;
		for (auto& ptr: *_entities.load(std::memory_order_relaxed))
		{
			if (*ptr == e)
			{
				_world.RemoveObject(&ptr->GetCollisionObject());
				(*_entities.load(std::memory_order_relaxed)).erase((*_entities.load(std::memory_order_relaxed)).begin() + ind);
				return;
			}
			ind++;
		}
	}

	void Scene::SetGravity(const geo::Vector& g) noexcept
	{
		_gravity = g;
	}

	void Scene::SetPhysicsUpdateFrequency(const unsigned short& ms) noexcept
	{
		_physicsUpdateFrequency = ms;
	}

	void Scene::StartPhysics() noexcept
	{
		_physicsIsActive.store(true, std::memory_order_relaxed);
	}

	void Scene::StopPhysics() noexcept
	{
		_physicsIsActive.store(false, std::memory_order_relaxed);
	}

	void Scene::Update(f64 dt) noexcept
	{
		if (!_started)
		{
			_started = true;
			StartPhysics();
		}
		//drawing all entities
		if (_display)
		{
			_display->Update();
			for (auto& ptr: *_entities.load(std::memory_order_relaxed))
			{
				if (!ptr->willDraw) continue;
				_display->Draw(ptr->sprite);
				ptr->Update();
			}
		}
	}
}