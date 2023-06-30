#include "SFML/Main.hpp"
#include "physics/Main.hpp"
#include <iostream>
#include <random>
#include <chrono>
#include <thread>
#define MAX std::numeric_limits<f64>::max()
#define MIN std::numeric_limits<f64>::min()
using namespace physics;
using namespace geo;

Collider* cldA[6];
Collider* cldB[6];
void Render(sf::RenderWindow* win, int a, int b)
{
    sf::Vector2i pixelPos = sf::Mouse::getPosition(*win);
    sf::Vector2f worldPos = win->mapPixelToCoords(pixelPos);
    if (a < 2)
    {
        f64 rad = !a ? 10 : 50;
        sf::CircleShape c(rad);
        c.setOrigin(rad, rad);
        c.setFillColor(sf::Color::Transparent);
        c.setOutlineColor(sf::Color::White);
        c.setOutlineThickness(1);
        c.setPosition(250, 250);
        win->draw(c);
    }
    if (b < 2)
    {
        f64 rad = !b ? 10 : 50;
        sf::CircleShape c(rad);
        c.setOrigin(rad, rad);
        c.setFillColor(sf::Color::Transparent);
        c.setOutlineColor(sf::Color::White);
        c.setOutlineThickness(1);
        c.setPosition(worldPos.x, worldPos.y);
        win->draw(c);
    }
    if (a == 2)
    {
        sf::RectangleShape r(sf::Vector2f(50, 50));
        r.setFillColor(sf::Color::Transparent);
        r.setOutlineColor(sf::Color::White);
        r.setOutlineThickness(1);
        r.setPosition(250, 250);
        win->draw(r);
    }
    if (b == 2)
    {
        sf::RectangleShape r(sf::Vector2f(50, 50));
        r.setFillColor(sf::Color::Transparent);
        r.setOutlineColor(sf::Color::White);
        r.setOutlineThickness(1);
        r.setPosition(worldPos.x, worldPos.y);
        win->draw(r);
    }
    if (a > 2)
    {
        std::vector<geo::Vector2> pts = ((PolygonCollider*)cldA[a])->GetPoints();
        sf::ConvexShape cvx(pts.size());
        for (int i = 0 ; i < pts.size(); i++)
            cvx.setPoint(i, sf::Vector2f(pts[i].x, pts[i].y));
        cvx.setFillColor(sf::Color::Transparent);
        cvx.setOutlineColor(sf::Color::White);
        cvx.setOutlineThickness(1);
        cvx.setPosition(250, 250);
        win->draw(cvx);
    }
    if (b > 2)
    {
        std::vector<geo::Vector2> pts = ((PolygonCollider*)cldB[b])->GetPoints();
        sf::ConvexShape cvx(pts.size());
        for (int i = 0 ; i < pts.size(); i++)
            cvx.setPoint(i, sf::Vector2f(pts[i].x, pts[i].y));
        cvx.setFillColor(sf::Color::Transparent);
        cvx.setOutlineColor(sf::Color::White);
        cvx.setOutlineThickness(1);
        cvx.setPosition(worldPos.x, worldPos.y);
        win->draw(cvx);
    }
    sf::CircleShape c(3);
    c.setFillColor(sf::Color::Red);
    Transform tA;
    tA.position.Set(250, 250);
    Transform tB;
    tB.position.Set(worldPos.x, worldPos.y);
    CollisionPoints result = cldA[a]->TestCollision(tA, cldB[b], tB);
    for (auto p : result.points)
    {
        c.setPosition(p.x - 3, p.y - 3);
        win->draw(c);
    }
}

int main(int argc, char** args)
{
    std::cout<<std::boolalpha;
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(500, 500), "Physics Engine", sf::Style::Default, settings);
    sf::View v = window.getView();
    v.setSize(500, -500);
    window.setView(v);
    cldA[0] = new CircleCollider(10);
    cldA[1] = new CircleCollider(50);
    cldA[2] = new BoxCollider(50, 50);
    cldA[3] = new PolygonCollider(geo::Vector2(), 20, 3);
    cldA[4] = new PolygonCollider(geo::Vector2(), 60, 3);
    cldA[5] = new PolygonCollider(geo::Vector2(), 8, 20);
    cldB[0] = new CircleCollider(10);
    cldB[1] = new CircleCollider(50);
    cldB[2] = new BoxCollider(50, 50);
    cldB[3] = new PolygonCollider(geo::Vector2(), 20, 3);
    cldB[4] = new PolygonCollider(geo::Vector2(), 60, 3);
    cldB[5] = new PolygonCollider(geo::Vector2(), 8, 20);
    int a = 0;
    int b = 0;
    int sz = 3;
    while (window.isOpen())
    {
        window.clear();
        sf::Event e;
        while (window.pollEvent(e))
        {
            if (e.type == sf::Event::Closed)
                window.close();
            if (e.type == sf::Event::KeyReleased)
            {
                if (e.key.code == sf::Keyboard::Q)
                    a = (a + 1) % 6;
                if (e.key.code == sf::Keyboard::W)
                    a = a ? a - 1 : 5;
                if (e.key.code == sf::Keyboard::A)
                    b = (b + 1) % 6;
                if (e.key.code == sf::Keyboard::S)
                    b = b? b - 1: 5;
                if (e.key.code == sf::Keyboard::T)
                    sz++;
                if (e.key.code == sf::Keyboard::Y)
                    sz--;
            }
        }
        Render(&window, a, b);
        window.display();
    }
}