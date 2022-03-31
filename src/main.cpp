#include "include/physics/Engine/Scene.hpp"
#include "include/SFML/Main.hpp"
#include "include/physics/Main.hpp"
#include <iostream>
#include <random>
using namespace physics;
using namespace geometry;
int main(int argc, char** args)
{
	std::cerr<<std::boolalpha;
	Scene s(Vector(0, 0), 30, 300, 300, "bruh");
	sf::RenderWindow* d = s.GetDisplay()->GetWindow();
	Softbody soft(20, 20);
	for (Spring spr: soft.springs)
	{
		std::cerr<<spr.a->position<<" "<<spr.b->position<<"\n";
		sf::Vertex line[2] =
		{
		    sf::Vertex(sf::Vector2f(10 + (spr.a->position.x * 12), 10 + (spr.a->position.y * 12))),
		    sf::Vertex(sf::Vector2f(10 + (spr.b->position.x * 12), 10 + (spr.b->position.y * 12)))
		};
		d->draw(line, 2, sf::Lines);
	}
	d->display();
	while (s.GetDisplay()->WindowIsOpen())
	{
		Time::Tick();
		s.Update(Time::deltaTime);
	}
}