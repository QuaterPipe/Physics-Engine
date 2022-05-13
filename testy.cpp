#include "src/include/geometry/main.hpp"
#include <iostream>
using namespace geo;

Vector intersection(Line la, Line lb) {
	// Line AB represented as a1x + b1y = c1
	double a = la.b.y - la.a.y;
	double b = la.a.x - la.b.x;
	double c = a*(la.a.x) + b*(la.a.y);
	// Line CD represented as a2x + b2y = c2
	double a1 = lb.b.y - lb.a.y;
	double b1 = lb.a.x - lb.b.x;
	double c1 = a1*(lb.a.x)+ b1*(lb.a.y);
	double det = a*b1 - a1*b;
	if (det == 0) {
		return Vector::Infinity;
	} else {
		double x = (b1*c - b*c1)/det;
		double y = (a*c1 - a1*c)/det;
		return Vector(x, y);
	}
}

int main()
{
	//Line l(Vector(31, 0), Vector(3, 0));
	//Line l2(Vector(4, 0), Vector(49, 0.5));
	//geo::Vector v = PointOfIntersect(l, l2);
	geo::Vector v = intersection(Line(Vector(0, 1), Vector(5, 0)), Line(Vector(2, 5), Vector(4, 8)));
	std::cout<<v.x<<" "<<v.y<<"\n";
}