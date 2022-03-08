#include "../include/physics/Scene.hpp"
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
			t.position = lerp(geometry::Vector(last.position), geometry::Vector(current.position), _accumulator / _s->physicsUpdateFrequency);
			object.SetTransform(t);
		}
		_accumulator += dt;
	}

	void Scene::Smoother::PhysicsUpdate() noexcept
	{
		_accumulator = 0;
	}

	Scene::Scene(const geometry::Vector& gravity) noexcept
	{
		_smoother = Smoother(*this);
		_gravity = gravity;
		std::string name = "display";
		display = new Display(300, 300, name);
		sf::View v = display->GetWindow()->getDefaultView();
		v.setSize(300, -300);
		display->SetView(v);
	}

	Scene::~Scene()
	{
		delete display;
	}

	const std::vector<std::unique_ptr<Entity>>& Scene::GetEntities() const noexcept
	{
		return _entities;
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
		_entities.emplace_back(ptr);
	}

	void Scene::RemoveEntity(Entity& e) noexcept
	{
		size_t ind;
		for (auto& ptr: _entities)
		{
			if (*ptr == e)
			{
				_world.RemoveObject(&ptr->GetCollisionObject());
				_entities.erase(_entities.begin() + ind);
				return;
			}
			ind++;
		}
	}

	void Scene::Update(f64 dt) noexcept
	{
		// getting the frames per second
		if (dt + _fpsCounter.total > 1000)
		{
			if (_fpsCounter.loopsPerSecond.size() > 60)
				_fpsCounter.loopsPerSecond.clear();
			_fpsCounter.loopsPerSecond.push_back((f64)_fpsCounter.loops);
			_fpsCounter.total = dt;
			_fpsCounter.loops = 0;
		}
		else
		{
			_fpsCounter.total += dt;
			_fpsCounter.loops++;
		}
		// updating physics
		if (physicsUpdateFrequency)
		{
			if (dt + _physicsUpdateCounter.total > 1000 / physicsUpdateFrequency)
			{
				_world.Update(_physicsUpdateCounter.total + dt);
				_world.ResolveCollisions(_physicsUpdateCounter.total + dt);
				_physicsUpdateCounter.total = 0;
				for (auto& ptr: _entities)
				{
					ptr->FixedUpdate();
					ptr->Update();
				}
			}
			else
			{
				_physicsUpdateCounter.total += dt;
			}
		}
		//drawing all entities
		if (display)
		{
			display->Update();
			for (auto& ptr: _entities)
			{
				display->Draw(ptr->GetSprite());
				ptr->Update();
			}
		}
	}
}