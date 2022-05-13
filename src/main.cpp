  #include "include/physics/Collision/Collision.hpp"
#include "include/physics/Collision/Rigidbody.hpp"
#include "include/physics/Tools/OstreamOverloads.hpp"
#include "include/physics/Engine/Time.hpp"
#include "include/physics/Main.hpp"
#include "include/SFML/Graphics.hpp"
#include "include/SFML/Window.hpp"
#include <iostream>
using namespace physics;
using namespace geo;

enum class Types
{
	BigSqr,
	MedSqr,
	SmlSqr,
	BigCirc,
	MedCirc,
	SmlCirc,
	BigTri,
	MedTri,
	SmlTri
};

Types& operator++(Types& t)
{
	t = static_cast<Types>((static_cast<int>(t) + 1) % static_cast<int>(Types::SmlTri));
	return t;
}

sf::RenderWindow window(sf::VideoMode(300, 300), "Collision testing");
std::vector<Rigidbody> objects;
std::vector<Collision> collisions;
Types left;
Types right;

void CheckCollisions()
{
	collisions.clear();
	for (Rigidbody& a: objects)
	{
		for (Rigidbody& b: objects)
		{
			if (a.collider->Equals(*b.collider))
				continue;
			CollisionPoints points = a.GetCollider().TestCollision(
				a.transform, &b.GetCollider(), b.transform
			);
			if (points.hasCollision)
			{
				Collision c;
				c.a = &a;
				c.b = &b;
				c.points = points;
				collisions.push_back(c);
				return;
			}
		}
	}
}

void UpdateTypes()
{
	switch (left)
	{
		case Types::BigSqr:
			objects[0].SetCollider(BoxCollider(100, 100));
			break;
		case Types::MedSqr:
			objects[0].SetCollider(BoxCollider(70, 50));
			break;
		case Types::SmlSqr:
		{
			PolygonCollider p;
			p.points.push_back(Vector(0, 0));
			p.points.push_back(Vector(90, 0));
			p.points.push_back(Vector(90, 40));
			p.points.push_back(Vector(75, 90));
			p.points.push_back(Vector(60, 40));
			p.points.push_back(Vector(45, 160));
			p.points.push_back(Vector(30, 40));
			p.points.push_back(Vector(15, 90));
			p.points.push_back(Vector(0, 40));
			objects[0].SetCollider(p);
			break;
		}
		case Types::BigCirc:
			objects[0].SetCollider(CircleCollider(100));
			break;
		case Types::MedCirc:
			objects[0].SetCollider(CircleCollider(50));
			break;
		case Types::SmlCirc:
			objects[0].SetCollider(CircleCollider(20));
			break;
		case Types::BigTri:
			objects[0].SetCollider(PolygonCollider(Vector(0, 0), Vector(0, 0), Vector(100, 0), Vector(50, 86.603)));
			break;
		case Types::MedTri:
			objects[0].SetCollider(PolygonCollider(Vector(0, 0), Vector(0, 0), Vector(35, 0), Vector(35, 60.622)));
			break;
		case Types::SmlTri:
			objects[0].SetCollider(PolygonCollider(Vector(0, 0), Vector(0, 0), Vector(20, 0), Vector(10, 17.321)));
			break;
	}
	switch (right)
	{
		case Types::BigSqr:
			objects[1].SetCollider(BoxCollider(100, 100));
			break;
		case Types::MedSqr:
			objects[1].SetCollider(BoxCollider(70, 50));
			break;
		case Types::SmlSqr:
			objects[1].SetCollider(BoxCollider(20, 20));
			break;
		case Types::BigCirc:
			objects[1].SetCollider(CircleCollider(100));
			break;
		case Types::MedCirc:
			objects[1].SetCollider(CircleCollider(50));
			break;
		case Types::SmlCirc:
			objects[1].SetCollider(CircleCollider(20));
			break;
		case Types::BigTri:
			objects[1].SetCollider(PolygonCollider(Vector(0, 0), Vector(0, 0), Vector(100, 0), Vector(50, 86.603)));
			break;
		case Types::MedTri:
			objects[1].SetCollider(PolygonCollider(Vector(0, 0), Vector(0, 0), Vector(35, 0), Vector(35, 60.622)));
			break;
		case Types::SmlTri:
			objects[1].SetCollider(PolygonCollider(Vector(0, 0), Vector(0, 0), Vector(20, 0), Vector(10, 17.321)));
			break;
	}
}

void Render()
{
	window.clear();
	for (Rigidbody& r: objects)
	{
		auto points = r.GetCollider().GetPoints(r.transform);
		if (points.size() > 1)
		{
			for (size_t i = 0; i < points.size(); i++)
			{
				sf::Vertex line[2] = {
					sf::Vertex(sf::Vector2f(points[i].x, points[i].y)),
					sf::Vertex(sf::Vector2f(points[(i + 1) % points.size()].x, points[(i + 1) % points.size()].y))
				};
				window.draw(line, 2, sf::Lines);
			}
		}
		else
		{
			CircleCollider c = dynamic_cast<CircleCollider&>(r.GetCollider());
			sf::CircleShape shape(c.radius);
			shape.setFillColor(sf::Color::Transparent);
			shape.setOutlineColor(sf::Color::White);
			shape.setOutlineThickness(2);
			shape.setPosition(r.position.x - c.radius, r.position.y - c.radius);
			window.draw(shape);
		}
	}
	for (Collision& c: collisions)
	{
		sf::Vertex line[2] = {
			sf::Vertex(sf::Vector2f(c.points.a.x, c.points.a.y), sf::Color::Green),
			sf::Vertex(sf::Vector2f(c.points.b.x, c.points.b.y), sf::Color::Green)
		};
		sf::CircleShape a(7);
		a.setPosition(c.points.a.x - 7, c.points.a.y - 7);
		a.setFillColor(sf::Color::Red);
		sf::CircleShape b(7);
		b.setPosition(c.points.b.x - 7, c.points.b.y - 7);
		b.setFillColor(sf::Color::Blue);
		window.draw(a);
		window.draw(b);
		window.draw(line, 2, sf::Lines);
	}
	window.display();
}
 
int main()
{
	f64 leftCoolDown = 0;
	f64 rightCoolDown = 0;
	Rigidbody r(BoxCollider(50, 50));
	r.position.Set(100, 100);
	Rigidbody r2(BoxCollider(50, 50));
	objects.push_back(r);
	objects.push_back(r2);
	sf::View v = window.getDefaultView();
	v.setSize(300, -300);
	window.setView(v);
	while (window.isOpen())
	{
		std::cout<<objects[1].position<<"\n";
		sf::Event e;
		Time::Tick();
		while (window.pollEvent(e))
		{
			if (e.type == sf::Event::Closed)
				window.close();
			if (e.type == sf::Event::MouseMoved)
			{
				sf::Vector2i pixelPos = sf::Mouse::getPosition(window);
				sf::Vector2f worldPos = window.mapPixelToCoords(pixelPos);
				objects[1].position.Set(worldPos.x, worldPos.y);
			}
			if (e.type == sf::Event::MouseButtonPressed)
			{
				if (e.mouseButton.button == sf::Mouse::Right)
				{
					rightCoolDown += 50;
					right = static_cast<Types>((static_cast<int>(right) + 1) % static_cast<int>(Types::SmlTri));
				}
				if (e.mouseButton.button == sf::Mouse::Left && !leftCoolDown)
				{
					leftCoolDown += 50;
					left = static_cast<Types>((static_cast<int>(left) + 1) % static_cast<int>(Types::SmlTri));
				}
				UpdateTypes();
			}
			if (e.type == sf::Event::KeyPressed)
			{
				if (e.key.code == sf::Keyboard::Space)
				{
					bool paused = true;
					while (paused)
					{
						while (window.pollEvent(e))
						{
							if (e.type == sf::Event::Closed)
							{
								window.close();
								paused = false;
								break;
							}
							if (e.type == sf::Event::KeyPressed && e.key.code == sf::Keyboard::Space)
							{
								paused = false;
								break;
							}
						}
						Render();
					}
				}
			}
		}
		if (leftCoolDown)
		{
			leftCoolDown -= Time::deltaTime;
			if (leftCoolDown < 0)
				leftCoolDown = 0;
		}
		if (rightCoolDown)
		{
			rightCoolDown -= Time::deltaTime;
			if (rightCoolDown < 0)
				rightCoolDown = 0;
		}
		CheckCollisions();
		Render();
	}
	return 0;
}