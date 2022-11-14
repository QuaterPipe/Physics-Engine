#include "include/SFML/Main.hpp"
#include "include/physics/Main.hpp"
#include <iostream>
#include <random>
#include <chrono>
#include <thread>

using namespace physics;
using namespace geo;
std::vector<Collision> collisions;


void CallBack(Collision& c, f64 dt)
{
    collisions.push_back(c);
}

int main()
{
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow    window(sf::VideoMode(500, 500), "Physics Engine", sf::Style::Default, settings);
    sf::View v = window.getView();
    v.setSize(500, -500);
    window.setView(v);
    PolygonCollider poly(BoxCollider(50, 50));
    Transform t;
    t.position.Set(250, 250);
    t.centerOfRotation = geo::Vector2(25, 25);
    bool pressing = false;
    f64 angle = 0;
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
            if (event.type == sf::Event::MouseButtonPressed)
                pressing = true;
            if (event.type == sf::Event::MouseButtonReleased)
                pressing = false;
        }
        if (pressing)
        {
            auto pos = window.mapPixelToCoords(sf::Vector2i(event.mouseButton.x, event.mouseButton.y));
            geo::Vector2 posVec(pos.x, pos.y);
            angle = (angle + 0.05);
            //angle = acos(posVec.Dot(geo::Vector2(250, 250)) /  (posVec.GetMagnitude() * 353.553391));
            t.rotation = geo::Matrix2(angle);
        }
        window.clear();
        sf::CircleShape c(3);
        for (geo::Vector2 vec: poly.points)
        {
            vec = t.TransformVector(vec);
            std::cout<<vec<<"\n";
            c.setPosition(vec.x - 3, vec.y - 3);
            window.draw(c);
        }
        c.setFillColor(sf::Color::Red);
        c.setPosition(247, 247);
        window.draw(c);
        window.display();
    }
}