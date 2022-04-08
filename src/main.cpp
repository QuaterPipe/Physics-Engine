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
	sf::Texture t;
	t.loadFromFile("bin/textures/Square.png");
	sf::Sprite square(t);
	sf::Sprite train;
	square.setScale(60, 0.6);
	Scene s(Vector(0, 0), 30, 300, 300, "bruh");
	sf::RenderWindow* d = s.GetDisplay()->GetWindow();
	Softbody soft(Transform(), 20, 20, Spring());
	soft.usesGravity = true;
	d->display();
	Transform tmpTrans;
	tmpTrans.position.Set(0, 270);
	BoxCollider b(Vector(0, 0), Vector(300, 30));
	Rigidbody rigid(tmpTrans, b, false, 5000);
	rigid.usesGravity = false;
	Entity rigidEntity("bruh", rigid, square);
	Entity softEntity("bruh", soft, train);
	s.AddEntity(rigidEntity);
	s.AddEntity(softEntity);
	Softbody& ref = dynamic_cast<Softbody&>(s.GetEntity(1).GetCollisionObject());
	ref.ApplyForce(Vector(5, 0), Vector(100, 100));
	while (s.GetDisplay()->WindowIsOpen())
	{
		Time::Tick();
		Softbody sft = dynamic_cast<Softbody&>(s.GetEntity(1).GetCollisionObject());
		for (Spring spr: sft.springs)
		{
			sf::Vertex line[2] =
			{
			    sf::Vertex(sf::Vector2f(10 + (spr.a->position.x * 12), 10 + (spr.a->position.y * 12))),
			    sf::Vertex(sf::Vector2f(10 + (spr.b->position.x * 12), 10 + (spr.b->position.y * 12)))
			};
			d->draw(line, 2, sf::Lines);
		}
		s.Update(Time::deltaTime);
	}
}