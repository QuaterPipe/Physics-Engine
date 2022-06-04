#pragma once
#include "Event.hpp"
#include <cstdint>
namespace physics::event
{
	class JoystickEvent : public Event
	{
		protected:
			JoystickEvent(const std::uint32_t& keyCode, const int& joystickNumber)
			: keyCode(keyCode), joystickNumber(joystickNumber) {};
		public:
			std::uint16_t keyCode;
			int joystickNumber;
			EVENT_CLASS_CATEGORY(EventCategory::Input | EventCategory::Joystick);
	};

	class JoystickMovedEvent : public JoystickEvent
	{
		public:
			f64 newX;
			f64 newY;
			JoystickMovedEvent(const int& joystickNumber, f64 newX, f64 newY)
			: JoystickEvent(0, joystickNumber), newX(newX), newY(newY)
			{}
			EVENT_CLASS_TYPE(JoystickMoved);
			virtual Event* Copy() const noexcept override {
				JoystickMovedEvent* e = new JoystickMovedEvent(joystickNumber, newX, newY);
				return e;
			}
	};

	class JoystickDownEvent : public JoystickEvent
	{
		public:
			JoystickDownEvent(const std::uint16_t& keyCode, const int& joystickNumber, const int& repeatCount)
			: JoystickEvent(keyCode, joystickNumber), repeatCount(repeatCount) {};
			int repeatCount;
			EVENT_CLASS_TYPE(JoystickDown);
			virtual Event* Copy() const noexcept override {
				JoystickDownEvent* e = new JoystickDownEvent(keyCode, joystickNumber, repeatCount);
				return e;
			}
	};

	class JoystickUpEvent : public JoystickEvent
	{
		public:
			JoystickUpEvent(const std::uint16_t& keyCode, const int& joystickNumber)
			: JoystickEvent(keyCode, joystickNumber) {};
			EVENT_CLASS_TYPE(JoystickUp);
			virtual Event* Copy() const noexcept override {
				JoystickUpEvent* e = new JoystickUpEvent(keyCode, joystickNumber);
				return e;
			}
	};

	class JoystickDisconnectedEvent : public Event
	{
		public:
			JoystickDisconnectedEvent(const int& joystickNumber)
			: joystickNumber(joystickNumber) {};
			int joystickNumber;
			EVENT_CLASS_CATEGORY(EventCategory::Input | EventCategory::Joystick);
			EVENT_CLASS_TYPE(JoystickDisconnected);
			virtual Event* Copy() const noexcept override {
				JoystickDisconnectedEvent* e = new JoystickDisconnectedEvent(joystickNumber);
				return e;
			}
	};

	class JoystickConnectedEvent : public Event
	{
		public:
			JoystickConnectedEvent(const int& joystickNumber)
				: joystickNumber(joystickNumber) {};
			int joystickNumber;
			EVENT_CLASS_CATEGORY(EventCategory::Input | EventCategory::Joystick);
			EVENT_CLASS_TYPE(JoystickConnected);
			virtual Event* Copy() const noexcept override {
				JoystickConnectedEvent* e = new JoystickConnectedEvent(joystickNumber);
				return e;
			}
	};
}