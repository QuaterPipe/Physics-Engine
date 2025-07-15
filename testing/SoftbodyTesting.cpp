#include <iostream>
#include "Testing.hpp"
using namespace geo;
using namespace physics;

#define WIN_WIDTH 1000
#define WIN_HEIGHT 500

void SoftbodyTest()
{
    sf::RenderWindow* win = NULL;
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(WIN_WIDTH, WIN_HEIGHT), "Physics Engine Demo", sf::Style::Default, settings);
    win = &window;
    sf::View v = window.getView();
    v.setSize(WIN_WIDTH, -WIN_HEIGHT);
    window.setView(v);
    PointMassSpring shpSpr;
    shpSpr.stiffness = 1e6;
    shpSpr.dampingFactor = 1e-9;
    shpSpr.restingLength = 1e-50;
    PointMassSpring spr;
    spr.stiffness = 1e9;
    spr.dampingFactor = 1e-13;
    spr.restingLength = 40;
    Softbody softbody(Transform(), 5, 5, spr, shpSpr, 0.01, 1e-8);
    softbody.transform.SetPosition(500, 50);
    Timer tm;
    bool lock = false;
    PointMass* mPoint = nullptr;
    while (window.isOpen())
    {
        sf::Event e;
        while (window.pollEvent(e))
        {
            if (e.type == sf::Event::Closed)
                window.close();
            if (e.type == sf::Event::MouseButtonPressed)
            {
                lock = true;
                sf::Vector2f worldPos = window.mapPixelToCoords(sf::Mouse::getPosition(window));
                geo::Vector2 vec(worldPos.x, worldPos.y);
                mPoint = softbody.GetClosestMassPoint(vec);
            }
            if (e.type == sf::Event::MouseButtonReleased)
                lock = false;
            if (e.type == sf::Event::KeyReleased)
            {
                if (e.key.code == sf::Keyboard::Space)
                {
                    softbody = Softbody(Transform(), 5, 5, spr, shpSpr, 0.01, 1e-20);
                    softbody.transform.SetPosition(500, 50);
                }
            }
        }

        if (lock)
        {
            sf::Vector2f worldPos = window.mapPixelToCoords(sf::Mouse::getPosition(window));
            geo::Vector2 vec(worldPos.x, worldPos.y);
            if (mPoint)
            {
                mPoint->position = vec - softbody.transform.GetPosition();
                std::cout << mPoint->velocity << "\n";
            }
        }
        softbody.shapeMatchingOn = false;
        f64 PE = 0;
        f64 KE = 0;
        for (auto& p : softbody.points)
            KE += p.KineticEnergy();
        for (auto& spr : softbody.springs)
        {
            PE += spr.PotentialEnergy();
            geo::Vector2 a = softbody.transform.TransformVector(spr.a->position);
            geo::Vector2 b = softbody.transform.TransformVector(spr.b->position);
            sf::Vertex line[2] = {
            sf::Vertex(sf::Vector2f(a.x, a.y), sf::Color::White),
            sf::Vertex(sf::Vector2f(b.x, b.y), sf::Color::White)
            };
            win->draw(line, 2, sf::Lines);
        }
        {
            std::cout << "PE: " << PE << " KE: " << KE << "\nTE: " << (KE + PE) << "\n";
            softbody.DerivePositionAndAngle();
            Transform ogToWorld;
            ogToWorld.SetPosition(softbody.derivedPos);
            ogToWorld.SetAngle(softbody.derivedAngle);
            for (auto& spr : softbody.springs)
            {
                geo::Vector2 tmpA = ogToWorld.TransformVector(softbody._originalShape[spr.aIndex].position);
                geo::Vector2 tmpB = ogToWorld.TransformVector(softbody._originalShape[spr.bIndex].position);
                sf::Vertex line[2] = {
                sf::Vertex(sf::Vector2f(tmpA.x + softbody.transform.GetPosition().x, tmpA.y + softbody.transform.GetPosition().y), sf::Color::Red),
                sf::Vertex(sf::Vector2f(tmpB.x + softbody.transform.GetPosition().x, tmpB.y + softbody.transform.GetPosition().y), sf::Color::Red)
                };
                win->draw(line, 2, sf::Lines);
            }
        }
        tm.Stop();
        for (int i = 0; i < 4; i++)
            softbody.Update(tm.deltaTime * 0.001, i);
        tm.Start();
        win->display();
        win->clear();
    }

}