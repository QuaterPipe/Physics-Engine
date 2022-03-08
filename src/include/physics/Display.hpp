#pragma once
#include "Entity.hpp"
#include "../SFML/Window.hpp"
#include "../SFML/Graphics.hpp"

namespace physics
{
	class Display
	{
		private:
			sf::RenderWindow _window;
			unsigned int _width;
			unsigned int _height;
			std::string _title;
		public:
			Display(unsigned int width, unsigned int height, std::string& title) noexcept;
			void Close() noexcept;
			void Draw(const Entity& e) noexcept;
			void Draw(const sf::Drawable& d) noexcept;
			unsigned int GetHeight() const noexcept;
			std::string GetTitle() const noexcept;
			unsigned int GetWidth() const noexcept;
			sf::RenderWindow* GetWindow() noexcept;
			const sf::RenderWindow& GetWindow() const noexcept;
			void SetHeight(unsigned int h) noexcept;
			void SetTitle(std::string windowName) noexcept;
			void SetWidth(unsigned int w) noexcept;
			void SetView(sf::View& view) noexcept;
			void Update(const bool& shouldClear=true) noexcept;
			bool WindowIsOpen() const noexcept;
	};
}