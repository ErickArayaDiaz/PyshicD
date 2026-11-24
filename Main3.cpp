// main.cpp - PysicD (PoC): ventana, loop con acumulador, entrada, escenas y limpieza.
#include <algorithm>
#include <cstddef>
#include <iomanip>
#include <iterator>
#include <optional>
#include <sstream>
#include <vector>

#include <SFML/Graphics.hpp>

#include "Config.h"
#include "MaterialTable.h"
#include "Renderer.h"
#include "Scenes.h"
#include "World.h"

int main()
{
    // ---- Declaración: el orden importa (se destruyen al revés) ----
    const Config        cfg{};
    const MaterialTable mats;
    World               world(cfg, mats);

    sf::RenderWindow ventana(sf::VideoMode({1000u, 700u}), "PysicD",
                             sf::Style::Titlebar | sf::Style::Close);
    ventana.setFramerateLimit(60);        // usa uno solo: límite o vsync
    ventana.setKeyRepeatEnabled(false);
    Renderer renderer(ventana, world);

    // ---- Tiempo ----
    constexpr double dt       = 1.0 / 60.0;
    constexpr double kMaxFrame = 0.1;                     // tope anti espiral
    const float      dtF      = static_cast<float>(dt);

    double    acc          = 0.0;
    double    simTime      = 0.0;
    long long pasosTotales = 0;
    int       maxPasos     = 0;                           // máximo desde el último refresco del título

    const double kEscalas[]   = {1.0, 0.5, 0.25, 0.1};    // solo 1x o más lento
    const int    kNumEscalas  = static_cast<int>(std::size(kEscalas));
    int          indiceEscala = 0;

    // ---- Estado de la aplicación ----
    bool pausaUsuario = false;
    bool sinFoco      = false;
    bool pasoUnico    = false;

    // Orden: Agua, Vapor, Piedra, Goma, Estatico
    const MaterialId kMateriales[] = {Mat::Agua, Mat::Vapor, Mat::Piedra, Mat::Goma, Mat::Estatico};
    int   indiceMaterial = 0;
    float radioActual    = 16.f;
    constexpr float kRadioMin = 8.f, kRadioMax = 40.f, kPasoRadio = 2.f;

    constexpr std::size_t kMaxDinamicos = 200;
    constexpr std::size_t kMaxTotal     = 600;            // cubre estáticos creados con la tecla 5

    // ---- Límites de la zona válida (la ventana es de tamaño fijo) ----
    constexpr float kMargenFuera = 200.f;
    const sf::Vector2u tam = ventana.getSize();
    const float W = static_cast<float>(tam.x), H = static_cast<float>(tam.y);
    const float xMin = -kMargenFuera, xMax = W + kMargenFuera;
    const float yMin = -kMargenFuera, yMax = H + kMargenFuera;
    long long eliminados = 0;

    // ---- Lambdas (declaradas después de las variables que capturan) ----
    auto contarDinamicos = [&]() {
        std::size_t n = 0;
        for (const auto& b : world.bodies())
            if (b->invMass != 0.f) ++n;
        return n;
    };

    auto solapa = [&](sf::Vector2f p, float r) {
        for (const auto& b : world.bodies())
        {
            const float dx = b->position.x - p.x;
            const float dy = b->position.y - p.y;
            const float s  = b->radius + r;
            if (dx * dx + dy * dy < s * s) return true;
        }
        return false;
    };

    auto puedeCrear = [&](sf::Vector2f p, float r) {
        return contarDinamicos() < kMaxDinamicos
            && world.bodies().size() < kMaxTotal
            && !solapa(p, r);
    };

    // Quita los dinámicos fuera de la zona válida. Devuelve cuántos quitó.
    auto limpiarFuera = [&]() -> std::size_t {
        std::vector<BodyId> fuera;                         // fase 1: recoger
        for (const auto& b : world.bodies())
        {
            if (b->invMass == 0.f) continue;               // paredes: nunca
            const float x = b->position.x, y = b->position.y;
            const bool dentro = x >= xMin && x <= xMax && y >= yMin && y <= yMax;
            if (!dentro) fuera.push_back(b->id);           // !dentro también atrapa NaN
        }
        const std::size_t antes = world.bodies().size();
        for (const BodyId id : fuera)                      // fase 2: borrar
            world.requestRemove(id);
        return antes - world.bodies().size();
    };

    Escena escenaActual = Escena::Caja;
    auto cambiarEscena = [&](Escena e) {
        escenaActual = e;
        cargarEscena(world, e, W, H);
        acc = 0.0; simTime = 0.0; pasosTotales = 0; eliminados = 0; maxPasos = 0;
    };

    // Un solo sitio para avanzar la física: evita desincronizar contadores.
    int pasos = 0;
    auto avanzar = [&]() {
        world.step(dtF);
        simTime += dt;
        ++pasos;
        ++pasosTotales;
    };

    // Fantasma de vista previa (no pertenece al mundo ni al Renderer)
    sf::CircleShape fantasma;
    fantasma.setFillColor(sf::Color::Transparent);
    fantasma.setOutlineThickness(1.f);

    cambiarEscena(Escena::Caja);

    sf::Clock reloj;
    sf::Clock titleClock;
    reloj.restart();                                       // justo antes del loop

    // ---- Loop principal: entrada -> step -> render ----
    while (ventana.isOpen())
    {
        pasos = 0;

        // 1) Eventos: solo cambian estado de la aplicación
        while (const std::optional<sf::Event> evento = ventana.pollEvent())
        {
            if (evento->is<sf::Event::Closed>())    ventana.close();
            if (evento->is<sf::Event::FocusLost>())   sinFoco = true;
            if (evento->is<sf::Event::FocusGained>()) sinFoco = false;

            if (const auto* k = evento->getIf<sf::Event::KeyPressed>())
            {
                switch (k->code)
                {
                case sf::Keyboard::Key::Escape: ventana.close(); break;
                case sf::Keyboard::Key::Space:  pausaUsuario = !pausaUsuario; break;
                case sf::Keyboard::Key::N:      pasoUnico = true; break;
                case sf::Keyboard::Key::Z: indiceEscala = std::min(indiceEscala + 1, kNumEscalas - 1); break;
                case sf::Keyboard::Key::X: indiceEscala = std::max(indiceEscala - 1, 0); break;
                case sf::Keyboard::Key::Num1: indiceMaterial = 0; break;
                case sf::Keyboard::Key::Num2: indiceMaterial = 1; break;
                case sf::Keyboard::Key::Num3: indiceMaterial = 2; break;
                case sf::Keyboard::Key::Num4: indiceMaterial = 3; break;
                case sf::Keyboard::Key::Num5: indiceMaterial = 4; break;
                case sf::Keyboard::Key::F1: cambiarEscena(Escena::Caja);   break;
                case sf::Keyboard::Key::F2: cambiarEscena(Escena::Anillo); break;
                case sf::Keyboard::Key::F3: cambiarEscena(Escena::Cuenco); break;
                case sf::Keyboard::Key::F4: cambiarEscena(Escena::Rampas); break;
                case sf::Keyboard::Key::R:  cambiarEscena(escenaActual);   break;
                default: break;
                }
            }

            if (const auto* w = evento->getIf<sf::Event::MouseWheelScrolled>())
                radioActual = std::clamp(radioActual + w->delta * kPasoRadio, kRadioMin, kRadioMax);

            if (const auto* m = evento->getIf<sf::Event::MouseButtonPressed>())
            {
                if (m->button == sf::Mouse::Button::Left)
                {
                    const sf::Vector2f p = ventana.mapPixelToCoords(m->position);
                    if (puedeCrear(p, radioActual))
                    {
                        const MaterialId mat = kMateriales[indiceMaterial];
                        const BodyId id = world.addCircle(Vector2(p.x, p.y), radioActual, mat);
                        if (id != INVALID_ID)
                            if (RigidBody* b = world.find(id))
                                if (mat == Mat::Vapor) b->gravityScale = -1.0f;   // el vapor sube
                    }
                }
            }
        }
        if (!ventana.isOpen()) break;                      // no seguir el frame tras cerrar

        // 2) Tiempo y pasos de física
        const bool   pausado = pausaUsuario || sinFoco;
        const double escala  = kEscalas[indiceEscala];
        const double frameTime = reloj.restart().asSeconds();   // siempre, con o sin pausa

        if (!pausado)
            acc += std::min(frameTime, kMaxFrame) * escala;

        while (acc >= dt)
        {
            acc -= dt;
            avanzar();
        }

        if (pausado && pasoUnico)                          // paso manual, fuera del acumulador
            avanzar();
        pasoUnico = false;                                 // se consume siempre

        maxPasos = std::max(maxPasos, pasos);
        eliminados += static_cast<long long>(limpiarFuera());

        // 3) Render
        ventana.clear(sf::Color(25, 25, 30));
        renderer.draw();

        if (ventana.hasFocus())
        {
            const sf::Vector2f c = ventana.mapPixelToCoords(sf::Mouse::getPosition(ventana));
            const bool rechazado = !puedeCrear(c, radioActual);
            fantasma.setOutlineColor(rechazado ? sf::Color(255, 70, 70, 200)
                                               : sf::Color(255, 255, 255, 160));
            fantasma.setRadius(radioActual);
            fantasma.setOrigin({radioActual, radioActual});
            fantasma.setPosition(c);
            ventana.draw(fantasma);
        }
        ventana.display();

        // 4) HUD en el título (~4 veces por segundo)
        if (titleClock.getElapsedTime().asSeconds() >= 0.25f)
        {
            std::ostringstream t;
            t << nombreEscena(escenaActual) << " | "
              << (pausado ? "PAUSA" : "corriendo") << " x" << escala
              << " | " << mats.get(kMateriales[indiceMaterial]).name
              << " r=" << std::fixed << std::setprecision(0) << radioActual
              << " | Pasos(max): " << maxPasos
              << " | Total: " << pasosTotales
              << " | Acc: " << std::setprecision(4) << acc
              << " | Sim: " << std::setprecision(2) << simTime
              << " | Din: " << contarDinamicos()
              << " | Elim: " << eliminados;
            ventana.setTitle(t.str());
            titleClock.restart();
            maxPasos = 0;
        }
    }
    return 0;
}