#include "include/physics/Engine/Scene.hpp"
#include "include/SFML/Main.hpp"
#include "include/physics/Main.hpp"
#include <iostream>
#include <random>
#include <chrono>

using namespace physics;
using namespace geo;
int main(int argc, char** args)
{
	sf::Texture sprites[5];
	sprites[0].loadFromFile("bin/textures/small_square.png");
	sprites[0].loadFromFile("bin/textures/medium_square.png");
	sprites[0].loadFromFile("bin/textures/big_square.png");
	sprites[0].loadFromFile("bin/textures/triangle.png");
	sprites[0].loadFromFile("bin/textures/circle.png");
	Scene scene(Vector(0, 0), 200, 300, 300, "Scene");
	sf::RenderWindow* window = scene.GetDisplay()->GetWindow();
	while (window->isOpen())
	{
		Time::Tick();
		scene.Update(Time::deltaTime);
		//if (sf::Mouse::)
	}
}