#include "Event.hpp"
#include <cstdint>
namespace physics::event
{
	class MouseMovedEvent : public Event
	{
		public:
			f64 x, y;
			MouseMovedEvent(const f64& x, const f64& y) noexcept : x(x), y(y) {};
			EVENT_CLASS_TYPE(MouseMoved);
			EVENT_CLASS_CATEGORY(EventCategory::Mouse | EventCategory::Input)
			virtual Event* Copy() const noexcept override {
				MouseMovedEvent* e = new MouseMovedEvent(x, y);
				return e;
			}
	};

	class MouseScrolledEvent : public Event
	{
		public:
			f64 xOffset, yOffset;
			MouseScrolledEvent(const f64& xOffset, const f64& yOffset) noexcept
			: xOffset(xOffset), yOffset(yOffset) {};
			EVENT_CLASS_TYPE(MouseScrolled);
			EVENT_CLASS_CATEGORY(EventCategory::Mouse | EventCategory::Input);
			virtual Event* Copy() const noexcept override {
				MouseScrolledEvent* e = new MouseScrolledEvent(xOffset, yOffset);
				return e;
			}
	};

	class MouseButtonEvent : public Event
	{
		protected:
			MouseButtonEvent(const f64& x, const f64& y, const int& mouseCode)
			: x(x), y(y), mouseCode(mouseCode) {};
		public:
			f64 x, y;
			int mouseCode;
			EVENT_CLASS_CATEGORY(EventCategory::Mouse | EventCategory::Input | EventCategory::MouseButton);
	};

	class MouseButtonDownEvent : public MouseButtonEvent
	{
		public:
			MouseButtonDownEvent(const f64& x, const f64& y, const int& mouseCode)
			: MouseButtonEvent(x, y, mouseCode) {};
			EVENT_CLASS_TYPE(MouseButtonDown);
			virtual Event* Copy() const noexcept override {
				MouseButtonDownEvent* e = new MouseButtonDownEvent(x, y, mouseCode);
				return e;
			}
	};

	class MouseButtonHeldEvent : public MouseButtonEvent
	{
		public:
			MouseButtonHeldEvent(const f64& x, const f64& y, const int& mouseCode, const f64& timeElapsed)
			: MouseButtonEvent(x, y, mouseCode), timeElapsed(timeElapsed) {};
			f64 timeElapsed;
			EVENT_CLASS_TYPE(MouseButtonHeld);
			virtual Event* Copy() const noexcept override {
				MouseButtonHeldEvent* e = new MouseButtonHeldEvent(x, y, mouseCode, timeElapsed);
				return e;
			}
	};

	class MouseButtonUpEvent : public MouseButtonEvent
	{
		public:
			MouseButtonUpEvent(const f64& x, const f64& y, const int& mouseCode)
			: MouseButtonEvent(x, y, mouseCode) {};
			EVENT_CLASS_TYPE(MouseButtonUp);
			virtual Event* Copy() const noexcept override {
				MouseButtonUpEvent* e = new MouseButtonUpEvent(x, y, mouseCode);
				return e;
			}
	};
}