#include <iostream>
#include <cstdlib>
#include <ctime>
#include "Testing.hpp"
#define WIN_WIDTH 1000
#define WIN_HEIGHT 500
#define PHYSICS_HERTZ 200.0
#define GRAVITY_SCALAR -160 * 4
using namespace physics;

enum class Type
{
    Circ,
    Poly,
    Pres,
    Soft
};

struct Object
{
    sf::CircleShape circ;
    sf::ConvexShape conv;
    Rigidbody* rigid = nullptr;
    Pressurebody* ball = nullptr;
    Softbody* soft = nullptr;
    Type t;
    sf::Color color;
};

void onClsn(CollisionManifold& c, f64 dt);
f64 Random(f64 l, f64 h);
void RenderObjects();
void CreateObject(Type type, sf::Vector2f pos, f64 rotVel);

int avgCounter = 0;
f64 frameAvg = 0;
bool hitBoxesOn = false;
bool frameSteppingOn = false;
bool canStep = false;
bool renderQuadtree = false;
sf::RenderWindow* win = NULL;
std::vector<Object> objects;
DynamicsWorld d(BoxCollider(Vector2(WIN_WIDTH * 0.5, WIN_HEIGHT * 0.5), Vector2(WIN_WIDTH, WIN_HEIGHT)));


void Demo()
{
    srand(time(NULL));
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(WIN_WIDTH, WIN_HEIGHT), "Physics Engine Demo", sf::Style::Default, settings);
    win = &window;
    sf::View v = window.getView();
    v.setSize(WIN_WIDTH, -WIN_HEIGHT);
    window.setView(v);
    d.SetCollisionCallBack(onClsn, 0);
    Time::Tick();
    Object floor;
    floor.rigid = new Rigidbody(PolygonCollider(BoxCollider(WIN_WIDTH * 0.8, 30)));
    floor.rigid->transform.SetPosition(WIN_WIDTH * 0.5, 40);
    floor.rigid->isStatic = true;
    floor.rigid->restitution = 1;
    floor.rigid->staticFriction = 1;
    floor.rigid->kineticFriction = 1;
    floor.rigid->transform.SetRotation(Radians(0));
    floor.t = Type::Poly;
    sf::ConvexShape c(4);
    for (int i = 0; i < 4; i++)
        c.setPoint(i, sf::Vector2f(floor.rigid->collider->GetPoints()[i].x, floor.rigid->collider->GetPoints()[i].y));
    c.setOutlineThickness(1);
    c.setOutlineColor(sf::Color::Black);
    c.setFillColor(sf::Color::Transparent);
    c.setOrigin(floor.rigid->transform.GetCOM().x, floor.rigid->transform.GetCOM().y);
    c.setPosition(250, 40);
    floor.conv = c;
    objects.push_back(floor);
    d.AddDynamicbody(floor.rigid);
    Timer tm;
    f64 accumulator = 0;
    f64 renderAccumulator = 0;
    while (window.isOpen())
    {
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
                if (e.mouseButton.button == sf::Mouse::Left)
                {
                    tm.Stop();
                    f64 x = tm.deltaTime < 1000 ? 0 : 1000 - tm.deltaTime;
                    for (int i = 0; i < 1; i++)
                    {
                        sf::Vector2f vec(worldPos.x + Random(-250, 250), worldPos.y + Random(-290, 290));
                        CreateObject(Type::Circ, vec, x * 0.1);
                    }
                }
                if (e.mouseButton.button == sf::Mouse::Right)
                {
                    tm.Stop();
                    f64 x = tm.deltaTime < 1000 ? 0 : 1000 - tm.deltaTime;
                    for (int i = 0; i < 1; i++)
                    {
                        sf::Vector2f vec(worldPos.x + Random(-15, 15), worldPos.y + Random(-15, 15));
                        CreateObject(Type::Poly, vec, x * 0.1);
                    }
                }
                if (e.mouseButton.button == sf::Mouse::Middle)
                {
                    tm.Stop();
                    f64 x = tm.deltaTime < 1000 ? 0 : 1000 - tm.deltaTime;
                    for (int i = 0; i < 1; i++)
                    {
                        sf::Vector2f vec(worldPos.x + Random(-15, 15), worldPos.y + Random(-15, 15));
                        CreateObject(Type::Pres, vec, x * 0.1);
                    }
                }
                if (e.mouseButton.button == sf::Mouse::XButton1)
                {
                    tm.Stop();
                    f64 x = tm.deltaTime < 1000 ? 0 : 1000 - tm.deltaTime;
                    for (int i = 0; i < 1; i++)
                    {
                        sf::Vector2f vec(worldPos.x + Random(-15, 15), worldPos.y + Random(-15, 15));
                        CreateObject(Type::Soft, vec, x * 0.1);
                    }
                }
            }
            if (e.type == sf::Event::KeyReleased)
            {
                if (e.key.code == sf::Keyboard::B)
                    hitBoxesOn = !hitBoxesOn;
                if (e.key.code == sf::Keyboard::N)
                    renderQuadtree = !renderQuadtree;
                if (e.key.code == sf::Keyboard::Space)
                {
                    frameSteppingOn = true;
                    canStep = true;
                }
                if (e.key.code == sf::Keyboard::LShift)
                {
                    frameSteppingOn = false;
                    canStep = false;
                }
            }
            if (e.type == sf::Event::KeyPressed)
            {
                if (e.key.code == sf::Keyboard::Space)
                {
                    frameSteppingOn = true;
                    canStep = true;
                }
            }
        }
        Time::Tick();

        if (frameSteppingOn)
        {
            if (canStep)
                d.Update(1.0 / PHYSICS_HERTZ);
            canStep = false;
        }
        if (!frameSteppingOn)
            accumulator += Time::deltaTime;
        if (accumulator >= 1.0 / PHYSICS_HERTZ * 1000.0 && !frameSteppingOn)
        {
            d.Update(1.0 / PHYSICS_HERTZ);
            accumulator -= 1.0 / PHYSICS_HERTZ * 1000;
        }
        renderAccumulator += Time::deltaTime;
        if (renderAccumulator >= 1.0 / 120.0 * 1000)
        {
            RenderObjects();
            renderAccumulator -= 1.0 / 120.0 * 1000;
        }
    }
}


f64 Random(f64 l, f64 h)
{
    f64 a = (f64)rand();
    a /= RAND_MAX;
    a = (h - l) * a + l;
    return a;
}


void onClsn(CollisionManifold& c, f64 dt)
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
        v[i].x += Random(-(width / 50.0), (width / 50.0));
        v[i].y += Random(-(width / 50.0), (width / 50.0));
    }
    *p = PolygonCollider(v);
}

void CreateObject(Type type, sf::Vector2f pos, f64 rotVel)
{
    Object obj;
    obj.t = type;
    Transform t;
    t.SetPosition(pos.x, pos.y);
    //t.SetRotation(Radians(Random(0, 360)));
    if (type == Type::Circ)
    {
        f64 rad = Random(5, 10);
        obj.rigid = new Rigidbody(CircleCollider(rad), t);
        obj.rigid->angularVelocity = rotVel;
        obj.rigid->SetMass(rad * 30);
        obj.rigid->SetInertia(obj.rigid->GetMass() * 1000);
        obj.rigid->gravity.Set(0, GRAVITY_SCALAR);
        sf::CircleShape c;
        c.setRadius(rad);
        c.setOutlineThickness(1);
        sf::Color col = sf::Color(Random(30, 255), Random(30, 255), Random(30, 255));
        c.setFillColor(sf::Color::Transparent);
        c.setOutlineColor(col);
        obj.circ = c;
        objects.push_back(obj);
        d.AddDynamicbody(obj.rigid);
    }
    if (type == Type::Poly)
    {
        f64 e = Random(3, 7);
        PolygonCollider p(BoxCollider(e * 10, e * 10));
        CreatePoly(&p, e, 150 / e);
        obj.rigid = new Rigidbody(p, t);
        obj.rigid->angularVelocity = rotVel;
        obj.rigid->SetMass(e * 30);
        obj.rigid->SetInertia(obj.rigid->GetMass() * 1000);
        obj.rigid->gravity.Set(0, GRAVITY_SCALAR);
        sf::ConvexShape c(p.GetPointCount());
        for (size_t i = 0; i < p.GetPointCount(); i++)
            c.setPoint(i, sf::Vector2f(p.GetPoint(i).x, p.GetPoint(i).y));
        c.setOutlineThickness(1);
        sf::Color col = sf::Color(Random(30, 255), Random(30, 255), Random(30, 255));
        c.setFillColor(col);
        c.setOutlineColor(sf::Color::Transparent);
        c.setOrigin(obj.rigid->transform.GetCOM().x, obj.rigid->transform.GetCOM().y);
        obj.conv = c;
        objects.push_back(obj);
        d.AddDynamicbody(obj.rigid);
    }
    if (type == Type::Pres)
    {
        f64 rad = Random(5, 10);
        PointMassSpring s;
        s.stiffness = 2000;
        s.dampingFactor = 2 * sqrt(s.stiffness * 100 / 12);
        obj.ball = new Pressurebody(1000000, 1, 17, 100, s);
        obj.ball->restitution = 0;
        obj.ball->gravity.Set(0, GRAVITY_SCALAR);
        obj.ball->transform = t;
        obj.ball->restitution = 0.9;
        obj.ball->staticFriction = 0.1;
        obj.ball->kineticFriction = 0.01;
        obj.color = sf::Color(Random(30, 255), Random(30, 255), Random(30, 255));
        objects.push_back(obj);
        d.AddDynamicbody(obj.ball);
    }
    if (type == Type::Soft)
    {
        f64 length = Random(5, 7);
        PointMassSpring shpSpr;
        shpSpr.stiffness = 0;
        shpSpr.dampingFactor = 0;
        shpSpr.restingLength = 1e-50;
        PointMassSpring spr;
        spr.stiffness = 1;
        spr.dampingFactor = 0.1;
        spr.restingLength = 15;
        obj.soft = new Softbody(t, length, length, spr, shpSpr, 0.01, 1.0 / 100.0);
        obj.color = sf::Color(Random(30, 255), Random(30, 255), Random(30, 255));
        objects.push_back(obj);
        d.AddDynamicbody(obj.soft);
    }
}

void RenderTree(Quadtree *q)
{
    if (q->subnodes[0])
    {
        RenderTree(q->subnodes[0].get());
        RenderTree(q->subnodes[1].get());
        RenderTree(q->subnodes[2].get());
        RenderTree(q->subnodes[3].get());
    }
    else
    {
        sf::RectangleShape r(sf::Vector2f(q->rect.width, q->rect.height));
        r.setOrigin(q->rect.width * 0.5f, q->rect.height * 0.5f);
        r.setFillColor(sf::Color::Transparent);
        r.setOutlineColor(sf::Color::Black);
        r.setOutlineThickness(1);
        r.setPosition(q->rect.x, q->rect.y);
        win->draw(r);
    }
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
        else if (obj.t == Type::Poly)
        {
            obj.conv.setPosition(obj.rigid->transform.GetPosition().x + obj.rigid->transform.GetCOM().x, obj.rigid->transform.GetPosition().y + obj.rigid->transform.GetCOM().y);
            obj.conv.setRotation(Degrees(obj.rigid->transform.GetAngle()));
            // std::cout << obj.conv.getPosition().x << ' ' << obj.conv.getPosition().y << "\n";
            win->draw(obj.conv);
        }
        else if (obj.t == Type::Pres)
        {
            sf::CircleShape c;
            c.setRadius(2);
            c.setOrigin(2, 2);
            /*for (auto& p : obj.ball->GetPoints())
            {
                c.setPosition(p.position.x + obj.ball->transform.GetPosition().x, p.position.y + obj.ball->transform.GetPosition().y);
                c.setFillColor(obj.color);
                win->draw(c);
            }*/
            /*sf::ConvexShape conv(obj.ball->GetPoints().size());
            auto t = obj.ball->GetPoints();
            std::cout << conv.getPointCount() << t.size() << std::endl;
            for (int i = 0; i < conv.getPointCount(); i++)
            {
                conv.setPoint(i, sf::Vector2f(obj.ball->transform.GetPosition().x + t[i].position.x, obj.ball->transform.GetPosition().y + t[i].position.y));
            }
            conv.setFillColor(obj.color);
            win->draw(conv);*/
            for (auto& s : obj.ball->GetSprings())
            {
                sf::Vertex line[2] = {
                sf::Vertex(sf::Vector2f(s.a->position.x + obj.ball->transform.GetPosition().x, s.a->position.y + obj.ball->transform.GetPosition().y), obj.color),
                sf::Vertex(sf::Vector2f(s.b->position.x + obj.ball->transform.GetPosition().x, s.b->position.y + obj.ball->transform.GetPosition().y), obj.color)
                };
                win->draw(line, 2, sf::Lines);
            }
        }
        else if (obj.t == Type::Soft)
        {
            Transform ogToWorld;
            ogToWorld.SetPosition(obj.soft->derivedPos);
            ogToWorld.SetAngle(obj.soft->derivedAngle);
            for (auto& spr : obj.soft->springs)
            {
                Vector2 tmpA = ogToWorld.TransformVector(obj.soft->_originalShape[spr.aIndex].position);
                Vector2 tmpB = ogToWorld.TransformVector(obj.soft->_originalShape[spr.bIndex].position);
                sf::Vertex line[2] = {
                sf::Vertex(sf::Vector2f(tmpA.x + obj.soft->transform.GetPosition().x, tmpA.y + obj.soft->transform.GetPosition().y), obj.color),
                sf::Vertex(sf::Vector2f(tmpB.x + obj.soft->transform.GetPosition().x, tmpB.y + obj.soft->transform.GetPosition().y), obj.color)
                };
                win->draw(line, 2, sf::Lines);
            }
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
        if (hitBoxesOn)
        {
            if (!obj.rigid)
                continue;
            auto bx = obj.rigid->GetCollider().BoundingBox(obj.rigid->transform);
            sf::RectangleShape box(sf::Vector2f(bx.width, bx.height));
            box.setPosition(obj.rigid->transform.GetPosition().x, obj.rigid->transform.GetPosition().y);
            box.setFillColor(sf::Color::Transparent);
            box.setOutlineColor(sf::Color::Red);
            box.setOutlineThickness(1);
            box.setOrigin(bx.width / 2.0, bx.height / 2.0);
            win->draw(box);
        }
    }
    std::vector<size_t> indices;
    for (size_t i = 0; i < objects.size(); i++)
    {
        if (!objects[i].rigid)
        {
            if (objects[i].ball && objects[i].ball->transform.GetPosition().y < -60)
                indices.insert(indices.begin(), i);
            if (objects[i].soft && objects[i].soft->transform.GetPosition().y < -60)
                indices.insert(indices.begin(), i);
        }
        else if (objects[i].rigid->transform.GetPosition().y < -60)
            indices.insert(indices.begin(), i);
    }
    for (size_t i : indices)
    {
        if (objects[i].rigid)
        {
            d.RemoveDynamicbody(objects[i].rigid);
            delete objects[i].rigid;
        }
        else if (objects[i].ball)
        {
            d.RemoveDynamicbody(objects[i].ball);
            delete objects[i].ball;
        }
        else if (objects[i].soft)
        {
            d.RemoveDynamicbody(objects[i].soft);
            delete objects[i].soft;
        }
        objects.erase(objects.begin() + i);
    }
    if (renderQuadtree)
        RenderTree(&d.quadtree);
    win->display();
    win->clear(sf::Color::White);
}