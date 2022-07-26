#include "include/SFML/Main.hpp"
#include "include/physics/Main.hpp"
#include <iostream>
#include <random>
#include <chrono>

using namespace physics;
using namespace geo;
int main(int argc, char** args)
{
	DynamicsWorld world;
	Rigidbody base(BoxCollider(500, 50));
	base.usesGravity = false;
	base.position.Set(0, 450);
	world.AddRigidbody(base);
	sf::RenderWindow window(sf::VideoMode(500, 500), "Physics Engine");
	Time::Tick();
	while (window.isOpen())
	{
		Time::Tick();
		sf::Event e;
		while (window.pollEvent(e))
		{
			if (e.type == sf::Event::Closed)
				window.close();
			if (e.type == sf::Event::MouseButtonPressed)
			{
				Rigidbody tmp(BoxCollider(50, 50));
				auto pos = window.mapPixelToCoords(sf::Vector2i(e.mouseButton.x, e.mouseButton.y));
				tmp.position.Set(pos.x, pos.y);
				tmp.SetMass(1000000);
				world.AddRigidbody(tmp);
			}
		}
		world.Update(Time::deltaTime);
		window.clear(sf::Color::Black);
		for (auto& r: world.GetRigidbodies())
		{
			BoxCollider b = (BoxCollider&)r.GetCollider();
			sf::RectangleShape rect(sf::Vector2f(b.width, b.height));
			rect.setPosition(r.position.x, r.position.y);
			rect.setRotation(Vector2(1, 1).Angle(r.rotation * Vector2(1, 1)));
			window.draw(rect);
		}
		window.display();
	}
}