#include <iostream>
#include "Testing.hpp"
using namespace physics;

#define WIN_WIDTH 1000
#define WIN_HEIGHT 500

RK4State sA, sB;
PointMass massA, massB;

void IntegrateA(f64 dt, int step)
{
    switch (step)
    {
        case 0:
            sA.a1 = massA.ComputeForce(massA.position, massA.velocity);
            sA.k1X = massA.velocity;
            sA.k1V = sA.a1;
            break;
        case 1:
            sA.tmpX = massA.position + 0.5 * dt * sA.k1X;
            sA.tmpV = massA.velocity + 0.5 * dt * sA.k1V;
            sA.a2 = massA.ComputeForce(sA.tmpX, sA.tmpV);
            sA.k2X = sA.tmpV;
            sA.k2V = sA.a2;
            break;
        case 2:
            sA.tmpX = massA.position + 0.5 * dt *sA.k2X;
            sA.tmpV = massA.velocity + 0.5 * dt *sA.k2V;
            sA.a3 = massA.ComputeForce(sA.tmpX, sA.tmpV);
            sA.k3X = sA.tmpV;
            sA.k3V = sA.a3;
            break;
        case 3:
            sA.tmpX = massA.position + dt * sA.k3X;
            sA.tmpV = massA.velocity + dt * sA.k3V;
            sA.a4 = massA.ComputeForce(sA.tmpX, sA.tmpV);
            sA.k4X = sA.tmpV;
            sA.k4V = sA.a4;
            massA.position += (dt / 6.0) * (sA.k1X + 2 * sA.k2X + 2 * sA.k3X + sA.k4X);
            massA.velocity += (dt / 6.0) * (sA.k1V + 2 * sA.k2V + 2 * sA.k3V + sA.k4V);
            massA.force.Set(0, 0);
            break;
    }
}

void IntegrateB(f64 dt, int step)
{
    switch (step)
    {
        case 0:
            sB.a1 = massB.ComputeForce(massB.position, massB.velocity);
            sB.k1X = massB.velocity;
            sB.k1V = sB.a1;
            break;
        case 1:
            sB.tmpX = massB.position + 0.5 * dt * sB.k1X;
            sB.tmpV = massB.velocity + 0.5 * dt * sB.k1V;
            sB.a2 = massB.ComputeForce(sB.tmpX, sB.tmpV);
            sB.k2X = sB.tmpV;
            sB.k2V = sB.a2;
            break;
        case 2:
            sB.tmpX = massB.position + 0.5 * dt *sB.k2X;
            sB.tmpV = massB.velocity + 0.5 * dt *sB.k2V;
            sB.a3 = massB.ComputeForce(sB.tmpX, sB.tmpV);
            sB.k3X = sB.tmpV;
            sB.k3V = sB.a3;
            break;
        case 3:
            sB.tmpX = massB.position + dt * sB.k3X;
            sB.tmpV = massB.velocity + dt * sB.k3V;
            sB.a4 = massB.ComputeForce(sB.tmpX, sB.tmpV);
            sB.k4X = sB.tmpV;
            sB.k4V = sB.a4;
            massB.position += (dt / 6.0) * (sB.k1X + 2 * sB.k2X + 2 * sB.k3X + sB.k4X);
            massB.velocity += (dt / 6.0) * (sB.k1V + 2 * sB.k2V + 2 * sB.k3V + sB.k4V);
            massB.force.Set(0, 0);
            break;
    }
}

void SpringTest()
{
    sf::RenderWindow* win = NULL;
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(WIN_WIDTH, WIN_HEIGHT), "Physics Engine Demo", sf::Style::Default, settings);
    win = &window;
    sf::View v = window.getView();
    v.setSize(WIN_WIDTH, -WIN_HEIGHT);
    window.setView(v);
    Timer tm;
    PointMassSpring spring;
    spring.stiffness = 10;
    spring.dampingFactor = 9;
    spring.restingLength = 40;
    massA.position.Set(250, 200);
    massB.position.Set(250, 100);
    massA.invMass = 0.1;
    massB.invMass = 0.1;
    spring.a = &massA;
    spring.b = &massB;
    massA.AddSpring(spring, true);
    massB.AddSpring(spring, false);
    /*std::cout << massB.ComputeForce(massB.position, massB.velocity) << std::endl;
    std::cout<<spring.CalculateForce();
    exit(0);*/
    //for (int i = 0; i < 10000000; i++)
    tm.Stop();
    tm.Start();
    while (window.isOpen())
    {
        sf::Event e;
        while (window.pollEvent(e))
        {
            if (e.type == sf::Event::Closed)
                window.close();
        }
        sf::Vertex line[2] = {
                sf::Vertex(sf::Vector2f(massA.position.x, massA.position.y), sf::Color::White),
                sf::Vertex(sf::Vector2f(massB.position.x, massB.position.y), sf::Color::Red)
        };
        win->draw(line, 12, sf::Lines);
        sf::CircleShape c(5);
        c.setOrigin(5, 5);
        c.setOutlineColor(sf::Color::Blue);
        c.setPosition(massA.position.x, massA.position.y);
        win->draw(c);
        c.setPosition(massB.position.x, massB.position.y);
        win->draw(c);
        f64 KE = massB.KineticEnergy() + massA.KineticEnergy();
        f64 PE = spring.PotentialEnergy();
        std::cout << "KE: " << KE << " PE: " << PE << "\n" << "TE: " << (KE + PE) << std::endl;
        std::cout << massB.ComputeForce(massB.position, massB.velocity)<<" " << (massA.position - massB.position).GetMagnitudeExact()<< std::endl;
        std::cout << massB.velocity << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl;
        //massA.force += Vector2(0, -9.81);
        //massB.force += Vector2(0, -9.81);
        tm.Stop();
        for (int i = 0; i < 4; i++)
        {
            IntegrateA(tm.deltaTime * 0.001, i);
            IntegrateB(tm.deltaTime * 0.001, i);
        }
        tm.Start();
        win->display();
        win->clear();
    }
}