#include "Testing.hpp"
#include <iostream>
#include <random>
#include <chrono>
#include <thread>
using namespace geo;
using namespace physics;
Collider* cldA[6];
Collider* cldB[6];
bool paused = false;
sf::Vector2f pos;
void Render(sf::RenderWindow* win, int a, int b)
{
    sf::Vector2i pixelPos = sf::Mouse::getPosition(*win);
    sf::Vector2f worldPos = win->mapPixelToCoords(pixelPos);
    if (paused)
        worldPos = pos;
    else
        pos = worldPos;
    Transform tB;
    tB.position.Set(worldPos.x, worldPos.y);
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
        for (int i = 0; i < pts.size(); i++)
            cvx.setPoint(i, sf::Vector2f(pts[i].x, pts[i].y));
        cvx.setFillColor(sf::Color::Transparent);
        cvx.setOutlineColor(sf::Color::White);
        cvx.setOutlineThickness(1);
        cvx.setPosition(250, 250);
        win->draw(cvx);
    }
    if (b > 2)
    {
        std::vector<geo::Vector2> pts = ((PolygonCollider*)cldB[b])->GetPoints(tB);
        sf::ConvexShape cvx(pts.size());
        for (int i = 0; i < pts.size(); i++)
            cvx.setPoint(i, sf::Vector2f(pts[i].x, pts[i].y));
        cvx.setFillColor(sf::Color::Transparent);
        cvx.setOutlineColor(sf::Color::White);
        cvx.setOutlineThickness(1);
        win->draw(cvx);
    }
    sf::CircleShape c(3);
    c.setFillColor(sf::Color::Red);
    Transform tA;
    tA.position.Set(250, 250);
    CollisionPoints result = cldA[a]->TestCollision(tA, cldB[b], tB);
    for (auto p : result.points)
    {
        sf::Vertex line[2] = {
            sf::Vertex(sf::Vector2f(p.x, p.y), sf::Color::Yellow),
            sf::Vertex(sf::Vector2f(p.x + result.normal.x * result.depth, p.y + result.normal.y * result.depth), sf::Color::Yellow)
        };
        c.setPosition(p.x - 3, p.y - 3);
        win->draw(c);
        win->draw(line, 2, sf::Lines);
    }
}
int size = 3;
void Render2(sf::RenderWindow* window)
{
    auto p = PolygonCollider(geo::Vector2(), 1000 / size, size);
    Transform tt;
    tt.position.Set(250, 250);
    sf::ConvexShape cvx(size);
    for (int i = 0; i < size; i++)
        cvx.setPoint(i, sf::Vector2f(p.GetPoint(i).x, p.GetPoint(i).y));
    cvx.setFillColor(sf::Color::Transparent);
    cvx.setOutlineColor(sf::Color::White);
    cvx.setOutlineThickness(1);
    cvx.setPosition(sf::Vector2f(250, 250));
    window->draw(cvx);
    sf::Vector2i pixelPos = sf::Mouse::getPosition(*window);
    sf::Vector2f worldPos = window->mapPixelToCoords(pixelPos);
    /*sf::Color colors[] = {
        sf::Color::Red,
        sf::Color::Yellow,
        sf::Color::Green,
        sf::Color::Blue
    };
    for (int i = 0; i < size; i++)
    {
        auto pt = tt.TransformVector(p.GetPoint(i));
        auto pt2 = tt.TransformVector(p.GetPoint((i + 1) % size));
        sf::Vertex line[2] = {
            sf::Vertex(sf::Vector2f(pt.x, pt.y), colors[i % (sizeof(colors) / sizeof(sf::Color))]),
            sf::Vertex(sf::Vector2f(pt2.x, pt2.y), colors[i % (sizeof(colors) / sizeof(sf::Color))])
        };
        window->draw(line, 2, sf::Lines);
    }*/
    for (int i = 0; i < 500; i++)
    {
        for (int j = 0; j < 500; j++)
        {
            if (p.Contains(Vector2(i, j), tt))
            {
                sf::CircleShape c(1);
                c.setFillColor(sf::Color::Red);
                c.setPosition(i, j);
                window->draw(c);
            }
        }
    }
}

void CollisionTest()
{
    std::cout << std::boolalpha;
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
    cldA[5] = new PolygonCollider(geo::Vector2(), 100, 20);
    cldB[0] = new CircleCollider(10);
    cldB[1] = new CircleCollider(50);
    cldB[2] = new BoxCollider(50, 50);
    cldB[3] = new PolygonCollider(geo::Vector2(), 20, 3);
    cldB[4] = new PolygonCollider(geo::Vector2(), 60, 3);
    cldB[5] = new PolygonCollider(geo::Vector2(), 100, 20);
    int a = 0;
    int b = 0;
    int sz = 3;
    // Render2(&window);
    window.display();
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
                    b = b ? b - 1 : 5;
                if (e.key.code == sf::Keyboard::T)
                {
                    size++;
                    window.clear();
                    Render2(&window);
                    window.display();
                }
                if (e.key.code == sf::Keyboard::Y && size > 3)
                {
                    size--;
                    window.clear();
                    Render2(&window);
                    window.display();
                }
                if (e.key.code == sf::Keyboard::Space)
                {
                    paused = !paused;
                }
            }
        }
        Render(&window, a, b);
        window.display();
    }
}