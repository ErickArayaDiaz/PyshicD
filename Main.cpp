// Demo visual 1: cuerpos cayendo sobre un suelo estatico (prueba del Renderer).
// Las pruebas numericas estan en Test.cpp.

#include <algorithm>
#include <optional>

#include <SFML/Graphics.hpp>

#include "Config.h"
#include "MaterialTable.h"
#include "Renderer.h"
#include "World.h"

namespace {
constexpr float kDt = 1.0f / 60.0f;
}

int main()
{
    // El orden de declaracion importa: se destruyen en orden inverso.
    const Config cfg{};
    const MaterialTable mats;
    World world(cfg, mats);
    sf::RenderWindow window(sf::VideoMode({800, 600}), "PysicD - prueba del Renderer");
    window.setFramerateLimit(60);
    Renderer renderer(window, world);

    // Suelo: circulo estatico enorme cuya parte alta queda en y = 550.
    world.addCircle({400.0f, 2550.0f}, 2000.0f, Mat::Estatico);

    struct Spawn { float x, y, r; MaterialId mat; };
    const Spawn spawns[] = {
        {390.0f,   40.0f, 15.0f, Mat::Agua},
        {405.0f,  -40.0f, 25.0f, Mat::Piedra},
        {395.0f, -120.0f, 20.0f, Mat::Goma},
        {410.0f, -200.0f, 18.0f, Mat::Goma},
        {300.0f,  500.0f, 12.0f, Mat::Vapor},   // sube
    };
    for (const Spawn& s : spawns)
    {
        const BodyId id = world.addCircle({s.x, s.y}, s.r, s.mat);
        if (s.mat == Mat::Vapor)
            if (RigidBody* b = world.find(id)) b->gravityScale = -1.0f;
    }

    const float maxFrame = 0.1f;   // tope de tiempo por frame (anti espiral)
    float acc = 0.0f;
    sf::Clock clock;

    while (window.isOpen())
    {
        while (const std::optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
                window.close();
        }

        acc += std::min(clock.restart().asSeconds(), maxFrame);
        while (acc >= kDt)
        {
            world.step(kDt);
            acc -= kDt;
        }

        window.clear(sf::Color(25, 25, 30));
        renderer.draw();
        window.display();
    }
    return 0;
}