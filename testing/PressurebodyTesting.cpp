#include <iostream>
#include "Testing.hpp"
using namespace geo;
using namespace physics;

#define WIN_WIDTH 1000
#define WIN_HEIGHT 500

void PressurebodyTest()
{
    sf::RenderWindow* win = NULL;
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(WIN_WIDTH, WIN_HEIGHT), "Physics Engine Demo", sf::Style::Default, settings);
    win = &window;
    sf::View v = window.getView();
    v.setSize(WIN_WIDTH, -WIN_HEIGHT);
    window.setView(v);
    PointMassSpring spr;
    spr.stiffness = 200;
    spr.dampingFactor = 180;
    spr.restingLength = 40;
    Pressurebody p(1000, 50, 12, 10, spr);
    p.transform.SetPosition(250, 100);
    Timer tm;
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
        f64 KE = 0;
        for (PointMass pt : p.GetPoints())
        {
            KE += pt.KineticEnergy();
            sf::CircleShape c(5);
            c.setOrigin(5, 5);
            c.setOutlineColor(sf::Color::Blue);
            c.setPosition(pt.position.x + p.transform.GetPosition().x, pt.position.y + p.transform.GetPosition().y);
            win->draw(c);
        }
        for (auto s : p.GetSprings())
        {
            sf::Vertex line[2] = {
                sf::Vertex(sf::Vector2f(s.a->position.x + p.transform.GetPosition().x, s.a->position.y + p.transform.GetPosition().y), sf::Color::White),
                sf::Vertex(sf::Vector2f(s.b->position.x + p.transform.GetPosition().x, s.b->position.y + p.transform.GetPosition().y), sf::Color::White)
            };
            win->draw(line, 2, sf::Lines);
        }
        f64 PE = 0;
        for (PointMassSpring sr : p.GetSprings())
            PE += sr.PotentialEnergy();
        std::cout << "KE: " << KE << " PE: " << PE << "\nTE: " << (KE + PE) << "--------------------------------------\n";
        //std::cout << p.GetVolume()<<" \ntm: ";
        tm.Stop();
        for (int i = 0; i < 4; i++)
        {
            p.Update(1.0 / 200.0, i);
        }
        //std::cout << tm.deltaTime << "\n";
        tm.Start();
        win->display();
        win->clear();
    }
}