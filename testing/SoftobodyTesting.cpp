#include "Testing.hpp"
using namespace geo;
using namespace physics;

#define WIN_WIDTH 1000
#define WIN_HEIGHT 500

void SoftbodyTest()
{
    sf::RenderWindow* win = NULL;
    DynamicsWorld d(BoxCollider(Vector2(WIN_WIDTH * 0.5, WIN_HEIGHT * 0.5), Vector2(WIN_WIDTH, WIN_HEIGHT)));
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(WIN_WIDTH, WIN_HEIGHT), "Physics Engine Demo", sf::Style::Default, settings);
    win = &window;
    sf::View v = window.getView();
    v.setSize(WIN_WIDTH, -WIN_HEIGHT);
    window.setView(v);
    Spring spr;
    Softbody s(Transform(), 10, 10, Spring());

}