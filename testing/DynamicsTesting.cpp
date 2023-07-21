#include "Testing.hpp"
#include <iostream>
using namespace geo;
using namespace physics;

void DynamicsTest()
{
	CircleCollider c(5);
	Transform ta;
	Transform tb;
	tb.position.Set(3, 0);
	Rigidbody a(c, ta);
	Rigidbody b(c, tb);
	b.velocity.Set(-0.1, 0);
	a.SetMass(1.0 / 0.09);
	b.SetMass(1.0 / 0.09);
	a.restitution = 0.1;
	b.restitution = 0.1;
	a.SetInertia(1.0 / 0.09);
	b.SetInertia(1.0 / 0.09);
	CollisionPoints cp = c.TestCollision(ta, &c, tb);
	// cp.points.erase(cp.points.begin() + 1);
	std::cout << cp.points[0]<< "\n";
	Collision col;
	col.a = &a;
	col.b = &b;
	col.points = cp;
	PhysicsSolver p;
	std::vector<Collision> vec;
	vec.push_back(col);
	p.Solve(vec, 1.0 / 60.0);
}