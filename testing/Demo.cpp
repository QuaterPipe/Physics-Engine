#include <iostream>
#include <cstdlib>
#include <ctime>
#include "Testing.hpp"
using namespace geo;
using namespace physics;


f64 Random(f64 l, f64 h)
{
    f64 a = (f64)rand();
    a /= RAND_MAX;
    a = (h - l) * a + l;
    return a;
}

enum class Type
{
    Circ,
    Poly
};

struct Object
{
    sf::CircleShape circ;
    sf::ConvexShape conv;
    Rigidbody* rigid = nullptr;
    Type t;
};

sf::RenderWindow* win = NULL;
std::vector<Object> objects;
DynamicsWorld d;

void onClsn(Collision& c, f64 dt)
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
        win->draw(circ);
        win->draw(line, 2, sf::Lines);
    }
}

void CreatePoly(PolygonCollider* p, size_t count, size_t width)
{
    *p = PolygonCollider(width, count);
    std::vector<Vector2> v = p->GetPoints();
    for (size_t i = 0; i < count; i++)
    {
        v[i].x += Random(-(width / 8.0), (width / 8.0));
        v[i].y += Random(-(width / 8.0), (width / 8.0));
    }
    *p = PolygonCollider(v);
}

void CreateObject(Type type, sf::Vector2f pos, f64 rotVel)
{
    Object obj;
    obj.t = type;
    Transform t;
    t.SetPosition(pos.x, pos.y);
    t.SetRotation(Radians(Random(0, 360)));
    if (type == Type::Circ)
    {
        f64 rad = rand() % 60 + 1;
        obj.rigid = new Rigidbody(CircleCollider(rad), t);
        obj.rigid->angularVelocity = rotVel;
        obj.rigid->SetMass(rad * 0.000005);
        obj.rigid->SetInertia(obj.rigid->GetMass() * 100);
        obj.rigid->restitution = 0.1;
        obj.rigid->kineticFriction = Random(0.6, 0.8);
        sf::CircleShape c;
        c.setRadius(rad);
        c.setOutlineThickness(1);
        c.setFillColor(sf::Color::Transparent);
        c.setOutlineColor(sf::Color(Random(30, 255), Random(30, 255), Random(30, 255)));
        obj.circ = c;
    }
    if (type == Type::Poly)
    {
        f64 e = Random(5, 80);
        PolygonCollider p(BoxCollider(e, e));
        CreatePoly(&p, Random(3, 7), e);
        obj.rigid = new Rigidbody(p, t);
        obj.rigid->angularVelocity = rotVel;
        obj.rigid->SetMass(0.0005 * e);
        obj.rigid->SetInertia(obj.rigid->GetMass() * 10000);
        obj.rigid->restitution = 0.1;
        obj.rigid->kineticFriction = Random(0.6, 0.8);
        sf::ConvexShape c(p.GetPointCount());
        for (size_t i = 0; i < p.GetPointCount(); i++)
            c.setPoint(i, sf::Vector2f(p.GetPoint(i).x, p.GetPoint(i).y));
        c.setOutlineThickness(1);
        c.setFillColor(sf::Color::Transparent);
        c.setOutlineColor(sf::Color(Random(30, 255), Random(30, 255), Random(30, 255)));
        c.setOrigin(obj.rigid->transform.GetCOM().x, obj.rigid->transform.GetCOM().y);
        obj.conv = c;
    }
    objects.push_back(obj);
    d.AddDynamicbody(obj.rigid);
}

void RenderObjects()
{
    for (auto obj : objects)
    {
        if (obj.t == Type::Circ)
        {
            obj.circ.setPosition(obj.rigid->transform.GetPosition().x - obj.circ.getRadius(), obj.rigid->transform.GetPosition().y - obj.circ.getRadius());
            win->draw(obj.circ);
        }
        else
        {
            obj.conv.setPosition(obj.rigid->transform.GetPosition().x + obj.rigid->transform.GetCOM().x, obj.rigid->transform.GetPosition().y + obj.rigid->transform.GetCOM().y);
            obj.conv.setRotation(geo::Degrees(obj.rigid->transform.GetAngle()));
            // std::cout << obj.conv.getPosition().x << ' ' << obj.conv.getPosition().y << "\n";
            win->draw(obj.conv);
        }
        if (obj.t == Type::Circ)
        {
            f64 rad = obj.circ.getRadius();
            Vector2 ang = GetVectorOnCircle(obj.rigid->transform.GetPosition(), rad, obj.rigid->transform.GetAngle());
            sf::Vertex line[2] = {
                sf::Vertex(sf::Vector2f(obj.rigid->transform.GetPosition().x, obj.rigid->transform.GetPosition().y)),
                sf::Vertex(sf::Vector2f(ang.x, ang.y))
            };
            win->draw(line, 2, sf::Lines);
        }
    }
    std::vector<size_t> indices;
    for (size_t i = 0; i < objects.size(); i++)
    {
        if (objects[i].rigid->transform.GetPosition().y < -60)
            indices.insert(indices.begin(), i);
    }
    for (size_t i : indices)
    {
        d.RemoveDynamicbody(objects[i].rigid);
        delete objects[i].rigid;
        objects.erase(objects.begin() + i);
    }
    win->display();
}

void Demo()
{
    srand(time(NULL));
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(500, 500), "Physics Engine Demo", sf::Style::Default, settings);
    win = &window;
    sf::View v = window.getView();
    v.setSize(500, -500);
    window.setView(v);
    d = DynamicsWorld();
    d.SetCollisionCallBack(onClsn, 0);
    Time::Tick();
    Object floor;
    floor.rigid = new Rigidbody(PolygonCollider(BoxCollider(400, 30)));
    floor.rigid->transform.SetPosition(250, 40);
    floor.rigid->isStatic = true;
    floor.rigid->restitution = 1;
    // floor.rigid->transform.SetRotation(geo::Radians(10));
    floor.t = Type::Poly;
    sf::ConvexShape c(4);
    for (int i = 0; i < 4; i++)
        c.setPoint(i, sf::Vector2f(floor.rigid->collider->GetPoints()[i].x, floor.rigid->collider->GetPoints()[i].y));
    c.setOutlineThickness(1);
    c.setFillColor(sf::Color::Transparent);
    c.setOrigin(floor.rigid->transform.GetCOM().x, floor.rigid->transform.GetCOM().y);
    c.setPosition(250, 40);
    floor.conv = c;
    objects.push_back(floor);
    d.AddDynamicbody(floor.rigid);
    for (int i = 0; i < 4; i++)
        std::cout << floor.rigid->transform.TransformVector(floor.rigid->collider->GetPoints()[i])<< " ";
    std::cout << "\n";
    Timer tm;
    while (window.isOpen())
    {
        window.clear();
        sf::Event e;
        while (window.pollEvent(e))
        {
            if (e.type == sf::Event::Closed)
                window.close();
            if (e.type == sf::Event::MouseButtonPressed)
            {
                tm.Reset();
                tm.Start();
            }
            if (e.type == sf::Event::MouseButtonReleased)
            {
                sf::Vector2i pixelPos = sf::Mouse::getPosition(window);
                sf::Vector2f worldPos = window.mapPixelToCoords(pixelPos);
                if (e.mouseButton.button == sf::Mouse::Right)
                {
                    tm.Stop();
                    f64 x = tm.deltaTime < 1000 ? 0 : 1000 - tm.deltaTime;
                    CreateObject(Type::Circ, worldPos, x * 0.1);
                }
                if (e.mouseButton.button == sf::Mouse::Left)
                {
                    tm.Stop();
                    f64 x = tm.deltaTime < 1000 ? 0 : 1000 - tm.deltaTime;
                    CreateObject(Type::Poly, worldPos, x * 0.1);
                }
            }
        }
        Time::Tick();
        d.Update(1 / 60.0); // delta time is measured in seconds by the physics engine
        // std::cout << Time::deltaTime << '\n';
        RenderObjects();
    }
}