#include <iostream>
#include "Testing.hpp"
using namespace physics;

#define WIN_WIDTH 1000
#define WIN_HEIGHT 500
#define PHYSICS_HERTZ 2000.0
#define WINDOW_HERTZ 120.0

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
    //anchor.isActive = false;
    anchor.isStatic = true;
    anchor.transform.SetPosition(500, 250);
    Rigidbody top(BoxCollider(10, 90));
    top.transform.SetPosition(500, 250);
    top.transform.SetRotation(Matrix2(Radians(-0.00)));
    Rigidbody bottom(BoxCollider(10, 90));
    //bottom.transform.SetRotation(Matrix2(Radians(1)));
    bottom.transform.SetPosition(500, 180);
    Rigidbody b2(BoxCollider(10, 90));
    b2.transform.SetPosition(500, 100);
    Rigidbody b3(BoxCollider(10, 90));
    b3.transform.SetPosition(500, 20);
    top.dragCoefficient = bottom.dragCoefficient = b2.dragCoefficient = b3.dragCoefficient = 0.0001;
    DistanceConstraint constraint2(&anchor, &top, 0, Vector2(0, 0), Vector2(0, 45));
   
    DistanceConstraint constraint(&top, &bottom, 0, Vector2(0, -40), Vector2(0, 40));
    
    DistanceConstraint constraint3(&bottom, &b2, 0, Vector2(0, -40), Vector2(0, 40));
    
    DistanceConstraint constraint4(&b2, &b3, 0, Vector2(0, -40), Vector2(0, 40));
    constraint.biasFactor = constraint2.biasFactor = constraint3.biasFactor = constraint4.biasFactor = 0.01;
    constraint.dampingFactor = constraint2.dampingFactor = constraint3.dampingFactor = constraint4.dampingFactor = 0.01;
    SliderConstraint slider(&top, Vector2(0, 0), Vector2(1, 1), Vector2(500, 250));
    slider.angleOffset = Radians(90);
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
    f64 accumulator = 0;
    f64 renderAccumulator = 0;
    f64 total = 0;
    size_t loops = 0;
    while (window.isOpen())
    {
        sf::Event e;
        while (window.pollEvent(e))
        {
            if (e.type == sf::Event::Closed)
                window.close();
            if (e.type == sf::Event::KeyReleased)
            {
                if (e.key.code == sf::Keyboard::Space)
                {
                    top.ApplyImpulse(Vector2(-3000, 200), Vector2(10, 10));
                }
                if (e.key.code == sf::Keyboard::D)
                {
                    top.ApplyImpulse(Vector2(10, 0));
                }
                if (e.key.code == sf::Keyboard::R)
                {
                    //constraint2.maxDistance += 1;
                }
            }
        }
        timer.Tick();
        accumulator += timer.deltaTime;
        if (accumulator >= (1.0 / PHYSICS_HERTZ) * 1000)
        {
            world.Update(1.0 / PHYSICS_HERTZ);
            accumulator -= (1.0 / PHYSICS_HERTZ) * 1000;
        }
        renderAccumulator += timer.deltaTime;
        if (renderAccumulator >= 1.0 / WINDOW_HERTZ * 1000)
        {
            sf::RectangleShape rect(sf::Vector2f(10, 90));
            rect.setOrigin(5, 45);
            rect.setFillColor(sf::Color::Transparent);
            rect.setOutlineColor(sf::Color::White);
            rect.setOutlineThickness(2);

            rect.setPosition(top.transform.GetPosition().x, top.transform.GetPosition().y);
            rect.setRotation(Degrees(top.transform.GetAngle()));
            win->draw(rect);

            rect.setPosition(bottom.transform.GetPosition().x, bottom.transform.GetPosition().y);
            rect.setRotation(Degrees(bottom.transform.GetAngle()));
            win->draw(rect);

            rect.setPosition(b2.transform.GetPosition().x, b2.transform.GetPosition().y);
            rect.setRotation(Degrees(b2.transform.GetAngle()));
            win->draw(rect);

            rect.setPosition(b3.transform.GetPosition().x, b3.transform.GetPosition().y);
            rect.setRotation(Degrees(b3.transform.GetAngle()));
            win->draw(rect);

            Vector2 v = top.transform.TransformVector(Vector2(0, 10));
            sf::CircleShape c(2);
            c.setPosition(500, 250);
            c.setOrigin(2, 2);
            win->draw(c);
            c.setPosition(v.x, v.y);
            c.setFillColor(sf::Color::Red);
            win->draw(c);
            win->display();
            win->clear();
            renderAccumulator -= 1.0 / WINDOW_HERTZ * 1000;
        }
        loops++;
        total += timer.deltaTime;
    }
    std::cout << (total / loops) << "\n";
}