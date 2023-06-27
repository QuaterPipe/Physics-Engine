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
	f64 mass = 0.2;
	if (argc - 1)
	{
		mass = std::stod(args[1]);
	}
	sf::Texture colours[5];
	colours[0].loadFromFile("bin/textures/red_square.png");
	colours[1].loadFromFile("bin/textures/cyan_square.png");
	colours[2].loadFromFile("bin/textures/green_square.png");
	colours[3].loadFromFile("bin/textures/magenta_square.png");
	colours[4].loadFromFile("bin/textures/lime_square.png");
	Scene scene(Vector(0, 0), 60, 300, 300, "Scene");
	sf::RenderWindow* display = scene.GetDisplay()->GetWindow();
	sf::Sprite boxes[5] = {
		sf::Sprite(colours[0]),
		sf::Sprite(colours[1]),
		sf::Sprite(colours[2]),
		sf::Sprite(colours[3]),
		sf::Sprite(colours[4])
	};
	boxes[0].setOrigin(25, 25);
	boxes[1].setOrigin(25, 25);
	boxes[2].setOrigin(25, 25);
	boxes[3].setOrigin(25, 25);
	boxes[4].setScale(3, 1);
	boxes[4].setOrigin(75, 25);
	Entity entities[5] = {
		Entity("Red", Rigidbody(BoxCollider(Vector(0, 0), Vector(50, 50))), boxes[0]),
		Entity("Blue", Rigidbody(BoxCollider(Vector(0, 0), Vector(50, 50))), boxes[1]),
		Entity("Green", Rigidbody(BoxCollider(Vector(0, 0), Vector(50, 50))), boxes[2]),
		Entity("Magenta", Rigidbody(BoxCollider(Vector(0, 0), Vector(50, 50))), boxes[3]),
		Entity("Lime", Rigidbody(BoxCollider(300, 50)), boxes[4])
	};
	entities[0].GetCollisionObject().position.Set(120, 490);
	entities[1].GetCollisionObject().position.Set(90, 250);
	entities[2].GetCollisionObject().position.Set(20, 300);
	entities[3].GetCollisionObject().position.Set(200, 300);
	entities[4].GetCollisionObject().position.Set(0, 0);
	dynamic_cast<Dynamicbody&>(entities[0].GetCollisionObject()).SetMass(mass);
	dynamic_cast<Dynamicbody&>(entities[1].GetCollisionObject()).SetMass(mass * 1.5);
	dynamic_cast<Dynamicbody&>(entities[2].GetCollisionObject()).SetMass(mass * 0.5);
	dynamic_cast<Dynamicbody&>(entities[3].GetCollisionObject()).SetMass(mass * 3);
	dynamic_cast<Dynamicbody&>(entities[0].GetCollisionObject()).SetInertia(mass);
	dynamic_cast<Dynamicbody&>(entities[1].GetCollisionObject()).SetInertia(mass);
	dynamic_cast<Dynamicbody&>(entities[2].GetCollisionObject()).SetInertia(mass);
	dynamic_cast<Dynamicbody&>(entities[3].GetCollisionObject()).SetInertia(mass);
	dynamic_cast<Dynamicbody&>(entities[4].GetCollisionObject()).SetMass(0);
	dynamic_cast<Dynamicbody&>(entities[4].GetCollisionObject()).isStatic = true;
	dynamic_cast<Dynamicbody&>(entities[0].GetCollisionObject()).restitution = 1;
	dynamic_cast<Dynamicbody&>(entities[1].GetCollisionObject()).restitution = 1;
	dynamic_cast<Dynamicbody&>(entities[2].GetCollisionObject()).restitution = 1;
	dynamic_cast<Dynamicbody&>(entities[3].GetCollisionObject()).restitution = 1;
	dynamic_cast<Dynamicbody&>(entities[4].GetCollisionObject()).restitution = 1;
	scene.AddEntity(entities[0]);
	scene.AddEntity(entities[1]);
	scene.AddEntity(entities[2]);
	scene.AddEntity(entities[3]);
	scene.AddEntity(entities[4]);
	while (display->isOpen())
	{
		Time::Tick();
		scene.Update(Time::deltaTime);
	}
	return 0;
}