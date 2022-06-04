#pragma once
#include "Event.hpp"
namespace physics::event
{

	class WindowClosedEvent : public Event
	{
		public:
			WindowClosedEvent() = default;
			EVENT_CLASS_TYPE(WindowClosed);
			EVENT_CLASS_CATEGORY(EventCategory::Window);
			virtual Event* Copy() const noexcept override {
				WindowClosedEvent* e = new WindowClosedEvent();
				return e;
			}
	};

	class WindowGainedFocusEvent : public Event
	{
		public:
			WindowGainedFocusEvent() = default;
			EVENT_CLASS_TYPE(WindowGainedFocus);
			EVENT_CLASS_CATEGORY(EventCategory::Window);
			virtual Event* Copy() const noexcept override {
				WindowGainedFocusEvent* e = new WindowGainedFocusEvent();
				return e;
			}
	};

	class WindowLostFocusEvent : public Event
	{
		public:
			WindowLostFocusEvent() = default;
			EVENT_CLASS_TYPE(WindowLostFocus);
			EVENT_CLASS_CATEGORY(EventCategory::Window);
			virtual Event* Copy() const noexcept override {
				WindowLostFocusEvent* e = new WindowLostFocusEvent();
				return e;
			}
	};

	class WindowResizedEvent : public Event
	{
		public:
			f64 newX, newY;
			WindowResizedEvent(const f64& newX, const f64& newY)
			: newX(newX), newY(newY) {};
			EVENT_CLASS_TYPE(WindowResized);
			EVENT_CLASS_CATEGORY(EventCategory::Window);
			virtual Event* Copy() const noexcept override {
				WindowResizedEvent* e = new WindowResizedEvent(newX, newY);
				return e;
			}
	};

	class WindowMovedEvent : public Event
	{
		public:
			f64 newX, newY;
			WindowMovedEvent(const f64& newX, const f64& newY)
			: newX(newX), newY(newY) {};
			EVENT_CLASS_TYPE(WindowMoved);
			EVENT_CLASS_CATEGORY(EventCategory::Window);
			virtual Event* Copy() const noexcept override {
				WindowMovedEvent* e = new WindowMovedEvent(newX, newY);
				return e;
			}
	};
}