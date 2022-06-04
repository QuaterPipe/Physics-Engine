#pragma once
#include "Event.hpp"

namespace physics::event
{
	class UpdateEvent : public Event
	{
		public:
			UpdateEvent(const f64& dt) : dt(dt) {};
			f64 dt;
			EVENT_CLASS_TYPE(Update);
			EVENT_CLASS_CATEGORY(EventCategory::Scene);
			virtual Event* Copy() const noexcept override {
				UpdateEvent* e = new UpdateEvent(dt);
				return e;
			}
	};

	class PhysicsUpdateEvent : public Event
	{
		public:
			PhysicsUpdateEvent(const f64& dt) : dt(dt) {};
			f64 dt;
			EVENT_CLASS_TYPE(PhysicsUpdate);
			EVENT_CLASS_CATEGORY(EventCategory::Scene);
			virtual Event* Copy() const noexcept override {
				PhysicsUpdateEvent* e = new PhysicsUpdateEvent(dt);
				return e;
			}
	};


	class RenderEvent : public Event
	{
		public:
			RenderEvent() = default;
			EVENT_CLASS_TYPE(Render);
			EVENT_CLASS_CATEGORY(EventCategory::Scene);
			virtual Event* Copy() const noexcept override {
				RenderEvent* e = new RenderEvent();
				return e;
			}
	};
}