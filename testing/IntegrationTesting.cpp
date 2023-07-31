#include "Testing.hpp"
#include <iostream>
using namespace physics;
using namespace geo;

void IntegrationTest()
{
	f64 Ax = 0, Av = 0, Aa = 0;
	f64 Bx = 0, Bv = 0, Ba = 0;
	f64 dt = 0.001;
	for (int i = 0; i < 1.0 / dt; i++)
	{
		Transform::SymplecticEulerIntegrate(&Ax, &Av, &Aa, dt);
		Transform::RK4Integrate(&Bx, &Bv, &Ba, dt);
		std::cout << "A pos: " << Ax << " B pos: " << Bx << "  A vel: " << Av << " B vel: " << Bv << "\n";
	}

}