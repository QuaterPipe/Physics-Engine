#include "../include/physics/Display.hpp"
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
			_window.draw(e.GetSprite());
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

	void Display::Update(const bool& shouldClear) noexcept
	{
		sf::Event e;
		while (_window.pollEvent(e))	
		{
			if (e.type == sf::Event::Closed)
			{
				_window.close();
			}
		}
		_window.display();
		if (shouldClear)
			_window.clear();
	}

	bool Display::WindowIsOpen() const noexcept
	{
		return _window.isOpen();
	}
}