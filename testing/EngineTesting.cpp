#include "Testing.hpp"
#include <iostream>
using namespace geo;
using namespace physics;
Rigidbody rigidbodies[10];
sf::RenderWindow* w = NULL;

void onCollision(Collision& c, f64 dt)
{
    sf::CircleShape circ(2);
    circ.setFillColor(sf::Color::Red);
    for (auto p : c.points.points)
    {
        circ.setPosition(p.x - 2, p.y - 2);
        w->draw(circ);
    }
}

f64 sis = 10000000;

void Render(void)
{
    sf::CircleShape cc(sis);
    cc.setPosition(250 - sis, (-sis + 40) - sis);
    cc.setFillColor(sf::Color::Transparent);
    cc.setOutlineThickness(1);
    w->draw(cc);

    //rect.setSize(sf::Vector2f(50, 50));
    std::cout << rigidbodies[1].velocity << "\n";
    for (int i = 1; i < 6; i++)
    {
        sf::CircleShape c(50);
        c.setPosition(rigidbodies[i].position.x - 50, rigidbodies[i].position.y - 50);
        c.setFillColor(sf::Color::Transparent);
        c.setOutlineThickness(1);
        Vector2 ang = GetVectorOnCircle(rigidbodies[i].position, 50, rigidbodies[i].rotation.Angle());
        sf::Vertex line[2] = {
            sf::Vertex(sf::Vector2f(rigidbodies[i].position.x, rigidbodies[i].position.y)),
            sf::Vertex(sf::Vector2f(ang.x, ang.y))
        };
        w->draw(line, 2, sf::Lines);
        w->draw(c);
    }
}

void EngineTest(void)
{
    std::cout << std::boolalpha;
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(500, 500), "Physics Engine", sf::Style::Default, settings);
    w = &window;
    sf::View v = window.getView();
    v.setSize(500, -500);
    window.setView(v);
    DynamicsWorld d;
    d.SetCollisionCallBack(onCollision, 0.16);
    rigidbodies[0] = Rigidbody(CircleCollider(sis), Transform(), false, PhysicsMaterial(), 1, false);
    rigidbodies[0].position.Set(250, -sis + 20);
    rigidbodies[0].isStatic = true;
    d.AddDynamicbody(&rigidbodies[0]);
    for (int i = 1; i < 6; i++)
    {
        rigidbodies[i] = Rigidbody(CircleCollider(50));
        rigidbodies[i].centerOfRotation.Set(25, 25);
        rigidbodies[i].position.Set(i * 10, 100);
        rigidbodies[i].SetMass(20e5);
        rigidbodies[i].SetInertia(20e5);
        rigidbodies[i].restitution = 0;
        d.AddDynamicbody(&rigidbodies[i]);
    }
    Time::Tick();
    while (window.isOpen())
    {
        window.clear();
        sf::Event e;
        while (window.pollEvent(e))
        {
            if (e.type == sf::Event::Closed)
                window.close();
        }
        Time::Tick();
        d.Update(1.0 / 60.0);
        std::cout << Time::deltaTime << "\n";
        Render();
        window.display();
    }
}