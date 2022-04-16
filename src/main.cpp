#include "include/physics/Engine/Scene.hpp"
#include "include/SFML/Main.hpp"
#include "include/physics/Main.hpp"
#include <iostream>
#include <random>
#include <chrono>

using namespace physics;
using namespace geometry;
int main(int argc, char** args)
{
	/*sf::Font futura;
	futura.loadFromFile("./bin/assets/Futura.tff");
	sf::Text fps;
	fps.setFont(futura);
	fps.setPosition(370, 50);
	fps.setCharacterSize(10);
	double force = 1;
	double mass = 1;
	double radius = 1;
	Spring sample;
	sample.dampingFactor = 0.1;
	sample.restingLength = 1;
	sample.stiffness = 0.2;
	if (argc - 1)
	{
		sample.restingLength = std::stod(args[1]);
		if (0 < argc - 2)
			sample.stiffness = std::stod(args[2]);
		if (0 < argc - 3)
			sample.dampingFactor = std::stod(args[3]);
		if (0 < argc - 4)
			force = std::stod(args[4]);
		if (0 < argc - 5)
			radius = std::stod(args[5]);
		if (0 < argc - 6)
			mass = std::stod(args[6]);
	}
	Softbody soft(Transform(), 10, 10, sample, sample.restingLength, radius, mass);
	soft.ApplyForce(Vector(1, 0) * force, Vector(5, 5));
	sf::RenderWindow window(sf::VideoMode(300, 300), "main");
	while (window.isOpen())
	{
		Time::Tick();
		//std::this_thread::sleep_for(std::chrono::milliseconds(100));
		sf::Event event;
		window.clear();
		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed)
			{
				window.close();
			}
		}
		
		if (sf::Mouse::isButtonPressed(sf::Mouse::Left))
		{
			soft.ApplyForce(Vector(1, 0) * force, Vector(sf::Mouse::getPosition(window).x, sf::Mouse::getPosition(window).y) / 7);
		}
		soft.ApplySpringForces();
		//soft.FixCollapsing();
		for (std::vector<MassPoint>& mVec: soft.points)
		{
			for (MassPoint& m: mVec)
			{
				m.velocity += m.force / m.mass;
				m.position += m.velocity;
				m.force.Set(0, 0);
			}
		}
		soft.UpdateCollider();
		window.clear();
		for (Spring spr: soft.springs)
		{
			sf::Vertex line[2] =
			{
			    sf::Vertex(sf::Vector2f(40 + (spr.a->position.x * 12), 40 + (spr.a->position.y * 12))),
			    sf::Vertex(sf::Vector2f(40 + (spr.b->position.x * 12), 40 + (spr.b->position.y * 12)))
			};
			window.draw(line, 2, sf::Lines);
		}
		fps.setString(std::to_string(Time::deltaTime) + "fps");
		window.draw(fps);
		window.display();
	}*/
	
	Scene scene(Vector(0, 0), 60, 300, 300, "Scene");
	sf::RenderWindow* display = scene.GetDisplay()->GetWindow();
	return 0;
}