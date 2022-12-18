#include "include/SFML/Main.hpp"
#include "include/physics/Main.hpp"
#include <iostream>
#include <random>
#include <chrono>
#include <thread>
#define MAX std::numeric_limits<f64>::max()
#define MIN std::numeric_limits<f64>::min()
using namespace physics;
using namespace geo;

int main(int argc, char** args)
{
    std::cout<<std::boolalpha;
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(500, 500), "Physics Engine", sf::Style::Default, settings);
    sf::View v = window.getView();
    v.setSize(500, -500);
    window.setView(v);
    Rigidbody r1(BoxCollider(50, 50));
    Rigidbody r2(CircleCollider(50));
    r1.position.Set(250, 250);
    while (window.isOpen())
    {
        window.clear();
        sf::Event e;
        while (window.pollEvent(e))
        {
            if (e.type == sf::Event::Closed)
                window.close();
        }
        sf::Vector2i pixelPos = sf::Mouse::getPosition(window);
        sf::Vector2f worldPos = window.mapPixelToCoords(pixelPos);
        r2.position.Set(worldPos.x, worldPos.y);
        sf::RectangleShape rect(sf::Vector2f(50, 50));
        sf::CircleShape cir(50);
        cir.setFillColor(sf::Color::Transparent);
        cir.setOutlineColor(sf::Color::White);
        cir.setOutlineThickness(1);
        rect.setPosition(sf::Vector2f(250, 250));
        rect.setFillColor(sf::Color::Transparent);
        rect.setOutlineColor(sf::Color::White);
        rect.setOutlineThickness(1);
        window.draw(rect);
        cir.setPosition(r2.position.x - 50, r2.position.y - 50);
        CollisionPoints c = r1.collider->TestCollision(r1.transform, r2.collider.get(), r2.transform);
        //rect.setRotation(45);
        window.draw(cir);
        sf::CircleShape circ(3);
        circ.setFillColor(sf::Color::Red);
        circ.setPosition(c.a.x - 3, c.a.y - 3);
        window.draw(circ);
        circ.setPosition(c.b.x - 3, c.b.y - 3);
        circ.setFillColor(sf::Color::Blue);
        window.draw(circ);
        //std::cout<<c.hasCollision<<"\r";
        window.display();
    }
}