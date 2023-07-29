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
        sf::Vertex line[2] = {
            sf::Vertex(sf::Vector2f(p.x, p.y), sf::Color::Yellow),
            sf::Vertex(sf::Vector2f(p.x + c.points.normal.x * c.points.depth, p.y + c.points.normal.y * c.points.depth), sf::Color::Yellow)
        };
        w->draw(circ);
        w->draw(line, 2, sf::Lines);
    }
}
bool tick = true, lock = false;

f64 sis = 10000000;

void Render(void)
{
    sf::RectangleShape rect(sf::Vector2f(500, 50));
    rect.setFillColor(sf::Color::Transparent);
    rect.setOutlineThickness(1);
    rect.setPosition(rigidbodies[0].position.x, rigidbodies[0].position.y);
    w->draw(rect);

    rect.setSize(sf::Vector2f(50, 50));
    rect.setOrigin(sf::Vector2f(25, 25));
    // std::cout << rigidbodies[1].velocity << "\n";
    for (int i = 1; i < 6; i++)
    {
        rect.setPosition(rigidbodies[i].position.x + 25, rigidbodies[i].position.y + 25);
        Vector2 ang = GetVectorOnCircle(rigidbodies[i].position, 50, rigidbodies[i].rotation.Angle());
        rect.setRotation(geo::Degrees(rigidbodies[i].rotation.Angle()));
        sf::Vertex line[2] = {
            sf::Vertex(sf::Vector2f(rigidbodies[i].position.x, rigidbodies[i].position.y)),
            sf::Vertex(sf::Vector2f(ang.x, ang.y))
        };

        w->draw(line, 2, sf::Lines);
        w->draw(rect);
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
    rigidbodies[0] = Rigidbody(BoxCollider(500, 50), Transform(), false, PhysicsMaterial(), 1, false);
    rigidbodies[0].position.Set(0, 2);
    rigidbodies[0].isStatic = true;
    rigidbodies[0].restitution = 0.9;
    d.AddDynamicbody(&rigidbodies[0]);
    for (int i = 1; i < 6; i++)
    {
        rigidbodies[i] = Rigidbody(BoxCollider(50, 50));
        rigidbodies[i].centerOfMass.Set(25, 25);
        rigidbodies[i].position.Set(200 + i * 1, 100);
        rigidbodies[i].SetMass(0.05);
        rigidbodies[i].SetInertia(5);
        rigidbodies[i].rotation.Set(fmod(i * 0.4123412382, M_PI * 2));
        rigidbodies[i].restitution = 0.9;
        d.AddDynamicbody(&rigidbodies[i]);
    }
    Time::Tick();
    while (window.isOpen())
    {
        window.clear();
        sf::Event e;
        lock = false;
        while (window.pollEvent(e))
        {
            if (e.type == sf::Event::Closed)
                window.close();
            if (e.type == sf::Event::KeyPressed && e.key.code == sf::Keyboard::Space)
                tick = !tick;
            if (e.type == sf::Event::KeyReleased && e.key.code == sf::Keyboard::F)
                lock = true;
        }
        Time::Tick();
        if (tick || lock)
            d.Update(1.0 / 60.0);
        // std::cout << Time::deltaTime << "\n";
        if (tick || lock)
        {
            Render();
            window.display();
        }
    }
}