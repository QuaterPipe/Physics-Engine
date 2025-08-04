#include <iostream>
#include "Testing.hpp"
using namespace geo;
using namespace physics;

#define WIN_WIDTH 1000
#define WIN_HEIGHT 500

void ConstraintTest()
{
    sf::RenderWindow* win = NULL;
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(WIN_WIDTH, WIN_HEIGHT), "Physics Engine Demo", sf::Style::Default, settings);
    win = &window;
    sf::View v = window.getView();
    v.setSize(WIN_WIDTH, -WIN_HEIGHT);
    window.setView(v);
    Rigidbody anchor(BoxCollider(1, 1));
    anchor.isActive = false;
    anchor.isStatic = true;
    anchor.transform.SetPosition(500, 250);
    Rigidbody top(BoxCollider(10, 90));
    top.transform.SetPosition(500, 250);
    top.transform.SetRotation(geo::Matrix2(geo::Radians(-0.00)));
    Rigidbody bottom(BoxCollider(10, 90));
    bottom.transform.SetRotation(geo::Matrix2(geo::Radians(1)));
    bottom.transform.SetPosition(500, 180);
    Rigidbody b2(BoxCollider(10, 90));
    b2.transform.SetPosition(500, 100);
    Rigidbody b3(BoxCollider(10, 90));
    b3.transform.SetPosition(500, 20);
    //b3.gravity.Set(0, -9000);
    DistanceConstraint constraint2(&anchor, &top, 0, geo::Vector2(0, 0), geo::Vector2(0, 45));
   
    DistanceConstraint constraint(&top, &bottom, 0, geo::Vector2(0, -40), geo::Vector2(0, 40));
    
    DistanceConstraint constraint3(&bottom, &b2, 0, geo::Vector2(0, -40), geo::Vector2(0, 40));
    
    DistanceConstraint constraint4(&b2, &b3, 0, geo::Vector2(0, -40), geo::Vector2(0, 40));
    constraint.biasFactor = constraint2.biasFactor = constraint3.biasFactor = constraint4.biasFactor = 0.01;
    constraint.dampingFactor = constraint2.dampingFactor = constraint3.dampingFactor = constraint4.dampingFactor = 0.01;
    SliderConstraint slider(&top, geo::Vector2(0, 0), geo::Vector2(1, 1), geo::Vector2(500, 250));
    slider.angleOffset = geo::Radians(90);
    slider.biasFactor = 0.01;
    DynamicsWorld world(BoxCollider(Vector2(WIN_WIDTH * 0.5, WIN_HEIGHT * 0.5), Vector2(WIN_WIDTH, WIN_HEIGHT)));
    world.AddDynamicbody(&top);
    world.AddDynamicbody(&bottom);
    world.AddDynamicbody(&b2);
    world.AddDynamicbody(&b3);

    world.AddConstraint(&constraint);
    world.AddConstraint(&constraint2);
    world.AddConstraint(&constraint3);
    world.AddConstraint(&constraint4);
    //world.AddConstraint(&slider);
    world.constraintIterationCount = 15;
    Timer timer;
    timer.Start();
    timer.Tick();
    while (window.isOpen())
    {
        f64 tke = top.KineticEnergy() + bottom.KineticEnergy() + b2.KineticEnergy() + b3.KineticEnergy();
        sf::Event e;
        while (window.pollEvent(e))
        {
            if (e.type == sf::Event::Closed)
                window.close();
            if (e.type == sf::Event::KeyReleased)
            {
                if (e.key.code == sf::Keyboard::Space)
                {
                    top.ApplyImpulse(geo::Vector2(-3, 2), geo::Vector2(10, 10));
                }
                if (e.key.code == sf::Keyboard::D)
                {
                    top.ApplyImpulse(geo::Vector2(10, 0));
                }
                if (e.key.code == sf::Keyboard::R)
                {
                    //constraint2.maxDistance += 1;
                }
            }
        }
        sf::RectangleShape rect(sf::Vector2f(10, 90));
        rect.setOrigin(5, 45);
        rect.setFillColor(sf::Color::Transparent);
        rect.setOutlineColor(sf::Color::White);
        rect.setOutlineThickness(2);

        rect.setPosition(top.transform.GetPosition().x, top.transform.GetPosition().y);
        rect.setRotation(geo::Degrees(top.transform.GetAngle()));
        win->draw(rect);

        rect.setPosition(bottom.transform.GetPosition().x, bottom.transform.GetPosition().y);
        rect.setRotation(geo::Degrees(bottom.transform.GetAngle()));
        win->draw(rect);

        rect.setPosition(b2.transform.GetPosition().x, b2.transform.GetPosition().y);
        rect.setRotation(geo::Degrees(b2.transform.GetAngle()));
        win->draw(rect);

        rect.setPosition(b3.transform.GetPosition().x, b3.transform.GetPosition().y);
        rect.setRotation(geo::Degrees(b3.transform.GetAngle()));
        win->draw(rect);

        geo::Vector2 v = top.transform.TransformVector(geo::Vector2(0, 10));
        sf::CircleShape c(2);
        c.setPosition(500, 250);
        c.setOrigin(2, 2);
        win->draw(c);
        c.setPosition(v.x, v.y);
        c.setFillColor(sf::Color::Red);
        win->draw(c);
        timer.Tick();
        world.Update(1.0 / 200.0);
        win->display();
        win->clear();
    }
}