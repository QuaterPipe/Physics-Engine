#include "../include/physics/Main/Scene.hpp"
#include "../include/physics/Main/Time.hpp"
#include <iostream>

namespace physics
{
	Scene::Smoother::Smoother(Scene& s) noexcept
	{
		_s = &s;
	}

	Scene::Smoother::Smoother() noexcept
	{
	}

	void Scene::Smoother::Update(f64 dt) noexcept
	{
		auto lerp = [&](geometry::Vector a, geometry::Vector b, f64 t){
			auto clamp = [&](f64 d){
				if (d < 0)
					return 0.0;
				else if (d > 1)
					return 1.0;
				else
					return d;
			};
			t = clamp(t);
			return geometry::Vector(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
		};
		for (auto& ptr: _s->GetEntities())
		{
			Transform t = ptr->GetCollisionObject().GetTransform();
			CollisionObject& object = ptr->GetCollisionObject();
			Transform last = object.GetLastTransform();
			Transform current = object.GetTransform();
			t.position = lerp(geometry::Vector(last.position), geometry::Vector(current.position), _accumulator / _s->_physicsUpdateFrequency);
			object.SetTransform(t);
		}
		_accumulator += dt;
	}

	void Scene::Smoother::PhysicsUpdate() noexcept
	{
		_accumulator = 0;
	}

	Scene::Scene(const geometry::Vector& gravity, unsigned short physicsUpdateHz, unsigned windowWidth, unsigned windowHeight, std::string windowTitle) noexcept
	: _physicsUpdateFrequency(physicsUpdateHz), _smoother(Smoother(*this)), _gravity(gravity), _display(new Display(windowWidth, windowHeight, windowTitle))
	{
		_ended.store(false, std::memory_order_relaxed);
		_entities.store(new std::vector<std::unique_ptr<Entity>>(), std::memory_order_relaxed);
		sf::View v = _display->GetWindow()->getDefaultView();
		// sfml displays y coordinates from top to bottom, so inverting the y value in view fixes this, displaying y coordinates from bottom to top.
		v.setSize(windowWidth, -windowHeight);
		_display->SetView(v);
		_physicsThread = std::thread(&Scene::_PhysicsLoop, this);
		StartPhysics();
	}

	Scene::~Scene()
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
		Entity* ptr = e.Clone();
		if (ptr->GetCollisionObject().IsDynamic())
		{
			_world.AddRigidbody(dynamic_cast<Rigidbody*>(&ptr->GetCollisionObject()));
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
		while (!_ended.load(std::memory_order_relaxed))
		{
			unsigned short hz = _physicsUpdateFrequency.load(std::memory_order_relaxed);
			while (!_physicsIsActive.load(std::memory_order_relaxed))
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(1 / hz * 1000));
			}
			timer.Start();
			_world.Update(hz);
			_world.ResolveCollisions(hz);
			for (auto& ptr: *_entities.load(std::memory_order_relaxed))
			{
				ptr->FixedUpdate();
				ptr->Update();
			}
			timer.Stop();
			if (timer.deltaTime > (1 / hz * 1000)) // calculations took too long
				continue;
			else
			{
				double d = (1 / hz * 1000) - timer.deltaTime;
				std::this_thread::sleep_for(std::chrono::milliseconds((int)d));
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

	void Scene::SetGravity(const geometry::Vector& g) noexcept
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
		//drawing all entities
		std::cerr<<"Updating!!! "<<(bool)_display<<"\n";
		if (_display)
		{
			_display->Update();
			for (auto& ptr: *_entities.load(std::memory_order_relaxed))
			{
				std::cerr<<"drawing!!\n";
				_display->Draw(ptr->GetSprite());
				ptr->Update();
			}
		}
	}
}