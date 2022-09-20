#include "include/SFML/Main.hpp"
#include "include/physics/Main.hpp"
#include <iostream>
#include <random>
#include <chrono>
#include <thread>

using namespace physics;
using namespace geo;
std::vector<Collision> collisions;

geo::Vector ProjectedGaussSeidelSolve (geo::Matrix matrix, geo::Vector right,
	f64 relaxation, int iterations, geo::Vector lo, geo::Vector hi)
{
	// Validation omitted
	auto x = right;
	f64 delta;
	// Gauss-Seidel with Successive OverRelaxation Solver
	for (int k = 0; k < iterations; ++k)
	{
		for (int i = 0; i < right.GetSize(); ++i)
		{
			delta = 0.0f;
			for (int j = 0; j < i; ++j)
				delta += matrix[i][j] * x[j];
			for (int j = i + 1; j < right.GetSize(); ++j)
				delta += matrix[i][j] * x[j];

			delta = (right[i] - delta) / matrix[i][i];
			x [i] += relaxation * (delta - x [i]);
			// Project the solution within the lower and higher limits
			if (x[i] < lo[i])
				x[i] = lo[i];
			if (x[i] > hi[i])
				x[i]=hi[i];
		}
	}
	return x;
}


void CallBack(Collision& c, f64 dt)
{
	//std::cout<<"my boy\n";
	collisions.push_back(c);
}
int main(int argc, char** args)
{
	DynamicsWorld world;
	Rigidbody base(BoxCollider(500, 50));
	base.usesGravity = false;
	base.position.Set(0, 50);
	base.isStatic = true;
	base.SetMass(0);
	world.AddRigidbody(base);
	std::function<void(Collision&, f64)> clmp = CallBack;
	world.SetCollisionCallBack(CallBack, 0);
	sf::ContextSettings settings;
	settings.antialiasingLevel = 8;
	sf::RenderWindow window(sf::VideoMode(500, 500), "Physics Engine", sf::Style::Default, settings);
	sf::View v = window.getView();
	v.setSize(500, -500);
	window.setView(v);
	Time::Tick();
	Timer timer;
	while (window.isOpen())
	{
		timer.Start();
		sf::Event e;
		while (window.pollEvent(e))
		{
			if (e.type == sf::Event::Closed)
				window.close();
			if (e.type == sf::Event::MouseButtonPressed)
			{
				Rigidbody tmp(BoxCollider(rand() % 100, rand() % 70));
				auto pos = window.mapPixelToCoords(sf::Vector2i(e.mouseButton.x, e.mouseButton.y));
				tmp.position.Set(pos.x, pos.y);
				tmp.SetMass((rand() % 1 + 3) * 1000);
				world.AddRigidbody(tmp);
			}
		}

		world.Update(1.0 / 600.0);
		window.clear(sf::Color::Black);
		for (auto& r: world.GetRigidbodies())
		{
			BoxCollider b = (BoxCollider&)r.GetCollider();
			sf::RectangleShape rect(sf::Vector2f(b.width, b.height));
			rect.setPosition(r.position.x, r.position.y);
			rect.setRotation(Vector2(1, 1).Angle(r.rotation * Vector2(1, 1)));
			rect.setFillColor(sf::Color(0, 0, 0, 0));
			rect.setOutlineColor(sf::Color(255, 255, 255));
			rect.setOutlineThickness(1);
			window.draw(rect);
		}
		for (auto& c: collisions)
		{
			for (auto& v: c.points.contactPoints)
			{
				sf::CircleShape circ(2);
				circ.setFillColor(sf::Color(210, 0, 0, 200));
				circ.setPosition(v.x - 2, v.y - 2);
				sf::Vertex line[2] = {
					sf::Vertex(sf::Vector2f(v.x, v.y)),
					sf::Vertex(sf::Vector2f(v.x - c.points.normal.x * 5, v.y - c.points.normal.y * 5))
				};
				window.draw(circ);
				window.draw(line, 2, sf::Lines);
			}
		}
		collisions.clear();
		window.display();
		timer.Stop();
		if (timer.deltaTime >= 1.0 / 600.0) // calculations took too long
			continue;
		else
		{
			double d = (1.f / (double)600 * 1000.f) - timer.deltaTime;
			std::this_thread::sleep_for(std::chrono::microseconds((int)(d * 1000)));
		}
		timer.Reset();
	}
}