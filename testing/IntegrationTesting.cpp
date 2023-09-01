#include "Testing.hpp"
#include <cstdlib>
#include <ctime>
#include <iostream>
using namespace physics;
using namespace geo;

void IntegrationTest()
{
	f64 Ax = 0, Av = 0, Aa = 0;
	f64 Bx = 0, Bv = 0, Ba = 0;
	f64 dt = 0.1;
	for (int i = 0; i < 1.0 / dt; i++)
	{
		Transform::SymplecticEulerIntegrate(&Ax, &Av, &Aa, dt);
		Transform::RK4Integrate(&Bx, &Bv, &Ba, dt);
		std::cout << "A pos: " << Ax << " B pos: " << Bx << "  A vel: " << Av << " B vel: " << Bv << "\n";
	}

	std::cout << "\n\n\n\n\n";
	Matrix2 a(M_PI / 2);
	std::cout << (M_PI / 2) << "\n" << a.Angle() << std::endl;

	Transform t;
	t.SetAngle(M_PI);
	t.SetScale(3, 3);
	t.SetPosition(-200, -493);
	t.SetCOM(Vector2(31, 31));
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			std::cout << t.GetTransformationMatrix()(i, j) << ' ';
		}
		std::cout << '\n';
	}
	for (int i = 0; i < 9; i++)
		std::cout << t.GetTransformationMatrix().arr[i] << ' ';
	std::cout << std::endl;
	Vector2 v(31, 3113);
	Vector3 v3(31, 3113, 1);
	v3 = t.GetTransformationMatrix() * v3;
	std::cout << "answer: " << v3 << "\n";
	geo::Vector2 tmp = v;
	std::cout << tmp << "\n";
	tmp = t.GetRotation() * tmp;
	std::cout << tmp << "\n";
	tmp = t.GetScale() * tmp;
	std::cout << tmp << "\n";
	tmp += t.GetPosition();
	std::cout << tmp << "\n";
	tmp.x += t.GetCOM().x - t.GetRotation()(0, 0) * t.GetCOM().x - t.GetRotation()(0, 1) * t.GetCOM().y;
	tmp.y += t.GetCOM().y - t.GetRotation()(1, 0) * t.GetCOM().x - t.GetRotation()(1, 1) * t.GetCOM().y;
	std::cout << tmp << "\n";
	std::cout << "\n----------------\n";
	tmp = v;
	std::cout << tmp << "\n";
	tmp -= t.GetCOM();
	std::cout << tmp << "\n";
	tmp.Rotate(geo::Vector2(0, 0), t.GetAngle());
	std::cout << tmp << "\n";
	tmp += t.GetCOM();
	std::cout << tmp << "\n";
	tmp *= t.GetScale();
	std::cout << tmp << "\n";
	tmp += t.GetPosition();
	std::cout << tmp << "\n";
	srand(time(0));
	int tests = 1'000'000;
	Matrix3 m = t.GetTransformationMatrix();
	Timer tim;
	tim.Start();
	for (int i = 0; i < tests; i++)
	{
		Vector3 vec(rand(), rand(), 1);
		vec = m * vec;
		Vector2 result(vec.x, vec.y);
	}
	tim.Stop();
	std::cout << tim.deltaTime << '\n';
	tim.Reset();
	tim.Start();
	for (int i = 0; i < tests; i++)
	{
		Vector2 vec(rand(), rand());
		vec = t.GetRotation() * vec;
		vec = t.GetScale() * vec;
		vec += t.GetPosition();
		vec.x += t.GetCOM().x - t.GetRotation()(0, 0) * t.GetCOM().x - t.GetRotation()(0, 1) * t.GetCOM().y;
		vec.y += t.GetCOM().y - t.GetRotation()(1, 0) * t.GetCOM().x - t.GetRotation()(1, 1) * t.GetCOM().y;
	}
	tim.Stop();
	std::cout << tim.deltaTime << '\n';
	int n;
	Matrix3 bb(Matrix2(M_PI / 2) * Matrix2(2, 0, 0, 2));
	Matrix3 cc(Matrix2(M_PI / 2) * Matrix2(20, 0, 0, 20));
	Timer tm;
	tm.Start();
	Matrix3 dd = bb * cc;
	tm.Stop();
	std::cout << "Time: " << (tm.deltaTime / 1000.0) << "\n";
	Matrix3 ee(Matrix2(M_PI) * Matrix2(40, 0, 0, 40));
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
			std::cout << bb(i, j) << ' ';
		std::cout << '\n';
	}
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
			std::cout << cc(i, j) << ' ';
		std::cout << '\n';
	}
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 2; j++)
			std::cout << dd(i, j) << ' ';
		std::cout << '\n';
	}
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
			std::cout << ee(i, j) << ' ';
		std::cout << '\n';
	}
	std::cout << std::endl;
	std::cin >> n;
}