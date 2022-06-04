#pragma once
#include "../../geometry/main.hpp"
#include <string>
namespace physics::event
{
	enum class EventType
	{
		None = 0,
		WindowClosed, WindowGainedFocus, WindowLostFocus, WindowResized, WindowMoved,
		Update, PhysicsUpdate, Render,
		KeyDown, KeyUp, KeyTyped,
		MouseButtonDown, MouseButtonUp, MouseButtonHeld, MouseMoved, MouseScrolled,
		JoystickDown, JoystickUp, JoystickMoved, JoystickConnected, JoystickDisconnected,
		EntityCreated, EntityDestroyed
	};

	enum EventCategory
	{
		None = 0,
		Window = BIT(0),
		Scene = BIT(1),
		Input = BIT(2),
		Keyboard = BIT(3),
		Mouse = BIT(4),
		MouseButton = BIT(5),
		Joystick = BIT(6),
		Entity = BIT(7)
	};

#define EVENT_CLASS_TYPE(type) static EventType GetStaticType() noexcept { return EventType::type; }\
	virtual EventType GetEventType() const noexcept override { return GetStaticType(); }

#define EVENT_CLASS_CATEGORY(category) virtual int GetCategoryFlags() const noexcept override { return category; }

	class EventDispatcher;

	class Event
	{
		friend class EventDispatcher;
		public:
			bool handled = false;
			virtual EventType GetEventType() const noexcept = 0;
			virtual int GetCategoryFlags() const noexcept = 0;
			bool IsInCategory(const EventCategory& category) const noexcept
			{ return GetCategoryFlags() & category; }
			virtual Event* Copy() const noexcept = 0;
	};

	/*class EventDispatcher
	{
		public:
			EventDispatcher(Event& event);
			template <typename T, typename F>
			bool Dispatch(const F& func)
			{
				if (_event.GetEventType() == T::GetStaticType())
				{
					_event.handled |= func(static_cast<T&>(_event));
					return true;
				}
				return false;
			}
		private:
			Event& _event;
	};*/
}