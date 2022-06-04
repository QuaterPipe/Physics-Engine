#pragma once
#include "Event.hpp"
#include <cstdint>
namespace physics::event
{
	class KeyEvent : public Event
	{
		protected:
			KeyEvent(const u_int16_t& keyCode) noexcept : keyCode(keyCode) {};
		public:
			u_int16_t keyCode;
			EVENT_CLASS_CATEGORY(EventCategory::Keyboard | EventCategory::Input);
	};

	class KeyDownEvent : public KeyEvent
	{
		public:
			u_int16_t repeatCount;
			KeyDownEvent(const int& keyCode, const u_int16_t repeatCount) noexcept
			: KeyEvent(keyCode), repeatCount(repeatCount) {}
			EVENT_CLASS_TYPE(KeyDown);
			virtual Event* Copy() const noexcept override {
				KeyDownEvent* e = new KeyDownEvent(keyCode, repeatCount);
				return e;
			}
	};

	class KeyUpEvent : public KeyEvent
	{
		public:
			KeyUpEvent(const int& keyCode) noexcept : KeyEvent(keyCode) {}
			EVENT_CLASS_TYPE(KeyUp);
			virtual Event* Copy() const noexcept override {
				KeyUpEvent* e = new KeyUpEvent(keyCode);
				return e;
			}
	};

	class KeyTypedEvent : public KeyEvent
	{
		public:
			KeyTypedEvent(const std::uint32_t& keyCode, const u_int16_t& repeatCount) noexcept
			: KeyEvent(keyCode), repeatCount(repeatCount) {}
			u_int16_t repeatCount;
			EVENT_CLASS_TYPE(KeyTyped);
			virtual Event* Copy() const noexcept override {
				KeyTypedEvent* e = new KeyTypedEvent(keyCode, repeatCount);
				return e;
			}
	};
}