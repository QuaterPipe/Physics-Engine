#pragma once
#include "Event.hpp"

namespace physics
{
	class Entity;
}
namespace physics::event
{
	class EntityCreatedEvent : public Event
	{
		public:
			class Entity* entity;
			EntityCreatedEvent(class Entity* entity) : entity(entity) {};
			EVENT_CLASS_CATEGORY(EventCategory::Entity);
			EVENT_CLASS_TYPE(EntityCreated);
			virtual Event* Copy() const noexcept override {
				EntityCreatedEvent* e = new EntityCreatedEvent(entity);
				return e;
			}
	};

	class EntityDestroyedEvent : public Event
	{
		public:
			class Entity* entity;
			EntityDestroyedEvent(class Entity* entity) : entity(entity) {};
			EVENT_CLASS_CATEGORY(EventCategory::Entity);
			EVENT_CLASS_TYPE(EntityCreated);
			virtual Event* Copy() const noexcept override {
				EntityDestroyedEvent* e = new EntityDestroyedEvent(entity);
				return e;
			}
	};
};