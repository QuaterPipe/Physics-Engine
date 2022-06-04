#include "../../include/physics/Engine/Display.hpp"
#include <iostream>
namespace physics
{
	Display::Display(unsigned int width, unsigned int height, std::string& title) noexcept
	{
		_width = width;
		_height = height;
		_title = title;
		_window.create(sf::VideoMode(width, height), title);
	}

	void Display::Close() noexcept
	{
		if (_window.isOpen())
			_window.close();
	}

	void Display::Draw(const Entity& e) noexcept
	{
		if (_window.isOpen())
			_window.draw(e.sprite);
	}

	void Display::Draw(const sf::Drawable& d) noexcept
	{
		if (_window.isOpen())
			_window.draw(d);
		
	}

	unsigned int Display::GetHeight() const noexcept
	{
		return _height;
	}

	std::string Display::GetTitle() const noexcept
	{
		return _title;
	}

	unsigned int Display::GetWidth() const noexcept
	{
		return _width;
	}

	sf::RenderWindow* Display::GetWindow() noexcept
	{
		return &_window;
	}

	const sf::RenderWindow& Display::GetWindow() const noexcept
	{
		return _window;
	}

	void Display::SetHeight(unsigned int h) noexcept
	{
		_height = h;
	}

	void Display::SetTitle(std::string windowName) noexcept
	{
		_title = windowName;
		_window.setTitle(_title);
	}

	void Display::SetWidth(unsigned int w) noexcept
	{
		_width = w;
	}

	void Display::SetView(sf::View& v) noexcept
	{
		_window.setView(v);
	}

	void Display::Update(event::EventListener& e, const bool& shouldClear) noexcept
	{
		std::vector<KeyRepeatPair> charsTyped;
		sf::Event ev;
		while (_window.pollEvent(ev))	
		{
			if (ev.type == sf::Event::Closed)
			{
				e.OnEvent(event::WindowClosedEvent());
				_window.close();
			}
			if (ev.type == sf::Event::Resized)
				e.OnEvent(event::WindowResizedEvent(ev.size.width, ev.size.height));
			if (ev.type == sf::Event::LostFocus)
				e.OnEvent(event::WindowLostFocusEvent());
			if (ev.type == sf::Event::GainedFocus)
				e.OnEvent(event::WindowGainedFocusEvent());
			if (ev.type == sf::Event::TextEntered)
			{
				int repeats = 0;
				for (auto& krp: keysTyped)
				{
					if (ev.text.unicode == krp.keyCode)
					{
						repeats = krp.repeats;
						krp.repeats++;
						break;
					}
				}
				charsTyped.emplace_back(ev.text.unicode, repeats, 0);
				if (!repeats)
					keysTyped.emplace_back(ev.text.unicode, 0, 0);
				e.OnEvent(event::KeyTypedEvent(ev.text.unicode, repeats));
			}
			if (ev.type == sf::Event::KeyPressed)
			{
				int repeats = 0;
				for (auto& krp: keysPressed)
				{
					if (ev.key.code == krp.keyCode)
					{
						repeats = krp.repeats;
						krp.repeats++;
						break;
					}
				}
				if (!repeats)
					keysPressed.emplace_back(ev.key.code, 0, 0);
				e.OnEvent(event::KeyDownEvent(ev.key.code, repeats));
			}
			if (ev.type == sf::Event::KeyReleased)
			{
				for (size_t i = 0; i < keysPressed.size(); i++)
				{
					if (keysPressed[i].keyCode == ev.key.code)
					{
						keysPressed.erase(keysPressed.begin() + i);
						break;
					}
				}
			}
			if (ev.type == sf::Event::KeyReleased)
				e.OnEvent(event::KeyUpEvent(ev.key.code));

			if (ev.type == sf::Event::MouseWheelMoved)
				e.OnEvent(event::MouseScrolledEvent(ev.mouseWheelScroll.x, ev.mouseWheelScroll.y));
			if (ev.type == sf::Event::MouseButtonPressed)
			{
				int repeats = 0;
				for (auto& krp: mouseButtonsPressed)
				{
					if (ev.mouseButton.button == krp.keyCode)
					{
						repeats = krp.repeats;
						krp.repeats++;
						break;
					}
				}
				if (!repeats)
					mouseButtonsPressed.emplace_back(ev.mouseButton.button, 0, 0);
				e.OnEvent(event::MouseButtonDownEvent(ev.mouseButton.x, ev.mouseButton.y, ev.mouseButton.button));
				if (repeats)
					e.OnEvent(event::MouseButtonHeldEvent(ev.mouseButton.x, ev.mouseButton.y, ev.mouseButton.button, repeats));
			}
			if (ev.type == sf::Event::MouseMoved)
			{
				e.OnEvent(event::MouseMovedEvent(ev.mouseMove.x, ev.mouseMove.y));
			}
			if (ev.type == sf::Event::MouseButtonReleased)
			{
				for (size_t i = 0; i < mouseButtonsPressed.size(); i++)
				{
					if (mouseButtonsPressed[i].keyCode == ev.mouseButton.button)
					{
						mouseButtonsPressed.erase(mouseButtonsPressed.begin() + i);
						break;
					}
				}
			}
			if (ev.type == sf::Event::JoystickButtonPressed)
			{
				int repeats = 0;
				for (auto& krp: joystickButtonsPressed)
				{
					if (ev.joystickButton.button == krp.keyCode
						&& ev.joystickButton.joystickId == krp.third)
					{
						repeats = krp.repeats;
						krp.repeats++;
						break;
					}
				}
				joystickButtonsPressed.emplace_back(ev.joystickButton.button, 0, ev.joystickButton.joystickId);
				e.OnEvent(event::JoystickDownEvent(ev.joystickButton.button, ev.joystickButton.joystickId, repeats));
			}
			if (ev.type == sf::Event::JoystickButtonReleased)
			{
				for (size_t i = 0; i < joystickButtonsPressed.size(); i++)
				{
					if (ev.joystickButton.button == joystickButtonsPressed[i].keyCode
						&& ev.joystickButton.joystickId == joystickButtonsPressed[i].third)
						joystickButtonsPressed.erase(joystickButtonsPressed.begin() + i);
				}
			}
			if (ev.type == sf::Event::JoystickMoved)
			{
				f64 x = ev.joystickMove.axis == sf::Joystick::X ? ev.joystickMove.position : 0;
				f64 y = ev.joystickMove.axis == sf::Joystick::Y ? ev.joystickMove.position : 0;
				e.OnEvent(event::JoystickMovedEvent(ev.joystickButton.joystickId, x, y));
			}
			if (ev.type == sf::Event::JoystickConnected)
			{
				e.OnEvent(event::JoystickConnectedEvent(ev.joystickButton.joystickId));
			}
			if (ev.type == sf::Event::JoystickDisconnected)
			{
				e.OnEvent(event::JoystickDisconnectedEvent(ev.joystickButton.joystickId));
			}
		}
		for (size_t i = keysTyped.size(); i != 0; i--)
		{
			if (std::find(charsTyped.begin(), charsTyped.end(), keysTyped[i]) == charsTyped.end())
				keysTyped.erase(keysTyped.begin() + i);
		}
		e.OnEvent(event::RenderEvent());
		_window.display();
		if (shouldClear)
			_window.clear();
	}

	bool Display::WindowIsOpen() const noexcept
	{
		return _window.isOpen();
	}
}