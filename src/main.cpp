#include "include/physics/Scene.hpp"
#include "include/SFML/Main.hpp"
#include "include/physics/Main.hpp"
#include <iostream>
#include <random>
using namespace physics;
using namespace geometry;
int main(int argc, char** args)
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
	t.position.Set(200, 50);
	rigid2.SetVelocity(Vector(-50, 0));
	rigid2.SetUsesGravity(false);
	rigid2.SetCollider(BoxCollider(Vector(0, 0), Vector(20, 20)));
	rigid2.SetTransform(t);
	rigid2.SetMass(50);
	std::cerr<<rigid1.GetRestitution()<<" "<<rigid1.GetStaticFriction()<<" "<<rigid1.GetKineticFriction()<<"\n";
	std::cerr<<rigid2.GetRestitution()<<" "<<rigid2.GetStaticFriction()<<" "<<rigid2.GetKineticFriction()<<"\n";
	Entity e2("Entity2", rigid2, box2);
	s.AddEntity(e1);
	s.AddEntity(e2);
	/*BoxCollider b(Vector(0, 0), Vector(20, 20));
	Transform t;
	t.position.Set(0, 0);
	BoxCollider b1(Vector(0, 0), Vector(20, 20));
	Transform t2;
	t2.position.Set(10, 10);
	CollisionPoints tmp = b.TestCollision(t, &b1, t2);
	std::cerr<<tmp.a<<" "<<tmp.b;
	BoxCollider b2(Vector(0, 0), Vector(20, 20));
	BoxCollider b3(Vector(0, 0), Vector(20, 20));*/
	Line l(Vector(0, 0), Vector(20, 0));
	Line l2(Vector(10, -10), Vector(10, 10));
	std::cerr<<VectorOfIntersect(l, l2);
	while (s.display->WindowIsOpen())
	{
		Time::Tick();
		//std::cout<<"Delta Time: "<<Time::deltaTime<<"\n";
		s.Update(Time::deltaTime);
	}
}