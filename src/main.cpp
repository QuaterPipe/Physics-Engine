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
	sf::Sprite box;
	sf::Texture square;
	square.loadFromFile("bin/textures/Square.png");
	box.setTexture(square);
	box.setOrigin(25, 25);
	Rigidbody rigid1;
	Transform t;
	t.position.Set(150, 50);
	rigid1.usesGravity = false;
	rigid1.transform = t;
	rigid1.SetCollider(BoxCollider(Vector(0, 0), Vector(50, 50)));
	//rigid1.SetAngularVelocity(1);
	rigid1.SetMass(5);
	Entity e1("Entity1", rigid1, box);
	Rigidbody rigid2;
	sf::Texture square2;
	square2.loadFromFile("bin/textures/Square.png");
	sf::Sprite box2;
	box2.setOrigin(10, 10);
	box2.setTexture(square2);
	box2.setTextureRect(sf::IntRect(0, 0, 20, 20));
	//box.setOrigin(10, 10);
	t.position.Set(300, 50);
	rigid2.velocity = Vector(-0.0005, 0);
	rigid2.usesGravity = false;
	rigid2.SetCollider(BoxCollider(Vector(0, 0), Vector(20, 20)));
	rigid2.transform = t;
	rigid2.SetMass(1);
	Entity e2("Entity2", rigid2, box2);
	s.AddEntity(e1);
	s.AddEntity(e2);
	s.SetPhysicsUpdateFrequency(5);
	while (s.GetDisplay()->WindowIsOpen())
	{
		Time::Tick();
		//std::cout<<"Delta Time: "<<Time::deltaTime<<"\n";
		s.Update(Time::deltaTime);
		//std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}
}