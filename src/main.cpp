#include "include/physics/Scene.hpp"
#include "include/SFML/Main.hpp"
#include "include/physics/Main.hpp"
#include <iostream>
#include <random>
using namespace physics;
using namespace geometry;
int main()
{
	std::cerr<<std::boolalpha;
	Scene s(Vector(0, 0));
	sf::Sprite box;
	sf::Texture square;
	square.loadFromFile("bin/textures/Square.png");
	box.setTexture(square);
	box.setOrigin(25, 25);
	Rigidbody rigid1;
	Transform t;
	t.position.Set(50, 50);
	rigid1.SetUsesGravity(false);
	rigid1.SetTransform(t);
	rigid1.SetCollider(BoxCollider(Vector::Origin, Vector(50, 50)));
	rigid1.SetMass(50);
	Entity e1("Entity1", rigid1, box);
	Rigidbody rigid2;
	box.scale(.4, .4);
	box.setOrigin(10, 10);
	t.position.Set(200, 50);
	rigid2.SetVelocity(Vector(-10, 0));
	rigid2.SetUsesGravity(false);
	rigid2.SetCollider(BoxCollider(Vector::Origin, Vector(20, 20)));
	rigid2.SetTransform(t);
	rigid2.SetMass(20);
	Entity e2("Entity2", rigid2, box);
	s.AddEntity(e1);
	s.AddEntity(e2);
	while (s.display->WindowIsOpen())
	{
		Time::Tick();
		s.Update(Time::deltaTime);
		std::cerr<<"huh??"<<((Rigidbody&)s.GetEntity(1).GetCollisionObject()).GetInvMass()<<"\n";
	}
}