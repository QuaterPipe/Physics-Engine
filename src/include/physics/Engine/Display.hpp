#pragma once
#include "Entity.hpp"
#include "../Events/EventListener.hpp"
#include "../../SFML/Window.hpp"

namespace physics
{
	class Display
	{
		private:
			sf::RenderWindow _window;
			unsigned int _width;
			unsigned int _height;
			std::string _title;
			struct KeyRepeatPair
			{
				u_int16_t keyCode;
				int repeats = 0;
				u_int16_t third = 0;
				KeyRepeatPair(u_int16_t k, int r, u_int16_t t)
				: keyCode(k), repeats(r), third(t) {}
				inline bool operator==(const KeyRepeatPair& krp) const noexcept
				{ return keyCode == krp.keyCode && repeats == krp.repeats;}
				inline bool operator !=(const KeyRepeatPair& krp) const noexcept
				{ return keyCode != krp.keyCode || repeats != krp.repeats;}
			
			};
			std::vector<KeyRepeatPair> keysTyped;
			std::vector<KeyRepeatPair> keysPressed;
			std::vector<KeyRepeatPair> mouseButtonsPressed;
			std::vector<KeyRepeatPair> joystickButtonsPressed;
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
			void Update(event::EventListener& e, const bool& shouldClear=true) noexcept;
			bool WindowIsOpen() const noexcept;
	};
}