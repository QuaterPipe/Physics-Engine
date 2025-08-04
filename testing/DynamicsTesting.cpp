#include "Testing.hpp"
#include <iostream>
using namespace physics;

void DynamicsTest()
{
	CircleCollider c(10);
	BoxCollider b(10, 10);
	Transform CTrans;
	Transform BTrans;
	CTrans.SetPosition(0, 10);
	Manifold pts = c.TestCollision(CTrans, &b, BTrans);
	std::cout << "hasCollision: " << pts.hasCollision << std::endl;
	std::cout << "normal: " << pts.normal << std::endl;
	std::cout << "depth: " << pts.depth << std::endl;
	for (auto p : pts.points)
		std::cout << "point: " << p << std::endl;
	DynamicsWorld world(BoxCollider(5000, 5000));
	Rigidbody rc(c, CTrans);
	rc.SetMass(1);
	rc.SetInertia(1);
	rc.restitution = 0;
	rc.velocity += Vector2(0, -1);
	rc.angularVelocity += 0.1;
	rc.staticFriction = 0;
	rc.kineticFriction = 0;
	Rigidbody rb(b, BTrans);
	rb.staticFriction = 0;
	rb.kineticFriction = 0;
	rb.isStatic = true;
	rc.restitution = 0;
	world.AddDynamicbody(&rc);
	world.AddDynamicbody(&rb);
	for (int i = 0; i < 100; i++)
		world.Update(1.0 / 100.0);
	std::cout << rc.velocity << std::endl;
	auto _rect = BoxCollider(1000, 1000);
	Vector2 center = _rect.GetCenter();
	f64 x = center.x;
	f64 y = center.y;
	f64 width = _rect.width * 0.5;
	f64 height = _rect.height * 0.5;

	f64 w = width * 0.5;
	f64 h = height * 0.5;

	BoxCollider SW(Vector2(x - w, y - h), Vector2(width, height));
	BoxCollider SE(Vector2(x + w, y - h), Vector2(width, height));
	BoxCollider NW(Vector2(x - w, y + h), Vector2(width, height));
	BoxCollider NE(Vector2(x + w, y + h), Vector2(width, height));
	std::cout << "Rect: " << _rect.pos << " " << _rect.dimensions << "\n";
	std::cout << "SW: " << SW.pos << " " << SW.dimensions << "\n";
	std::cout << "SE: " << SE.pos << " " << SE.dimensions << "\n";
	std::cout << "NW: " << NW.pos << " " << NW.dimensions << "\n";
	std::cout << "NE: " << NE.pos << " " << NE.dimensions << "\n";
}