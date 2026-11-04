// main.cpp de PRUEBAS (no es el main final del PoC).
// Parte 1: 5 verificaciones numericas de la fisica, sin ventana.
// Parte 2: una demo visual para ver el Renderer funcionando.
//
// Uso:  pysicd.exe            -> pruebas + ventana
//       pysicd.exe --tests    -> solo pruebas (sin ventana)

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>

#include <SFML/Graphics.hpp>

#include "Config.h"
#include "MaterialTable.h"
#include "Renderer.h"
#include "World.h"

namespace {

constexpr float kDt = 1.0f / 60.0f;
int g_fallos = 0;

void check(const char* nombre, float obtenido, float esperado, float tol)
{
    const bool ok = std::fabs(obtenido - esperado) <= tol;
    if (!ok) ++g_fallos;
    std::printf("  [%s] %-36s obtenido=%10.4f  esperado=%10.4f\n",
                ok ? "OK " : "MAL", nombre, obtenido, esperado);
}

void stepN(World& w, int n)
{
    for (int i = 0; i < n; ++i) w.step(kDt);
}

// 1) Aceleracion: caida libre con Euler simplectico.
//    v_n = g*n*dt   y   y_n = y0 + g*dt^2 * n(n+1)/2
void testCaidaLibre(const Config& cfg, const MaterialTable& mats)
{
    std::puts("\n1) Caida libre (agua, r=10, 60 pasos = 1 s)");
    World w(cfg, mats);
    const BodyId id = w.addCircle({0.0f, 0.0f}, 10.0f, Mat::Agua);
    stepN(w, 60);

    const RigidBody* b = w.find(id);
    const float n = 60.0f;
    const float g = cfg.gravity.y;
    check("velocidad y = g*n*dt", b->velocity.y, g * n * kDt, 0.5f);
    check("posicion y (simplectico)", b->position.y,
          g * kDt * kDt * n * (n + 1.0f) / 2.0f, 0.5f);
    std::printf("  (referencia continua 0.5*g*t^2 = %.4f: difiere por la discretizacion)\n",
                0.5f * g * 1.0f * 1.0f);
}

// 2) Un cuerpo estatico no se mueve aunque haya gravedad.
void testEstatico(const Config& cfg, const MaterialTable& mats)
{
    std::puts("\n2) Cuerpo estatico (debe avisar al crearse)");
    World w(cfg, mats);
    const BodyId id = w.addCircle({100.0f, 100.0f}, 30.0f, Mat::Estatico);
    stepN(w, 60);

    const RigidBody* b = w.find(id);
    check("posicion y sin cambio", b->position.y, 100.0f, 0.0001f);
    check("velocidad y = 0", b->velocity.y, 0.0f, 0.0001f);
}

// 3) La gravedad no depende de la masa ni del radio.
void testMasaNoImporta(const Config& cfg, const MaterialTable& mats)
{
    std::puts("\n3) Misma caida con distinta masa y radio (agua r=10 vs piedra r=20)");
    World w(cfg, mats);
    const BodyId ia = w.addCircle({0.0f, 0.0f}, 10.0f, Mat::Agua);
    const BodyId ip = w.addCircle({200.0f, 0.0f}, 20.0f, Mat::Piedra);
    stepN(w, 60);

    const RigidBody* a = w.find(ia);
    const RigidBody* p = w.find(ip);
    std::printf("  masa agua=%.2f  masa piedra=%.2f\n", a->mass, p->mass);
    check("diferencia de y", a->position.y - p->position.y, 0.0f, 0.01f);
    check("diferencia de velocidad y", a->velocity.y - p->velocity.y, 0.0f, 0.01f);
}

// 4) gravityScale negativo: el vapor sube.
void testVapor(const Config& cfg, const MaterialTable& mats)
{
    std::puts("\n4) Vapor con gravityScale = -1 (sube)");
    World w(cfg, mats);
    const BodyId id = w.addCircle({0.0f, 500.0f}, 10.0f, Mat::Vapor);
    w.find(id)->gravityScale = -1.0f;
    stepN(w, 60);

    const RigidBody* b = w.find(id);
    const float n = 60.0f;
    const float g = cfg.gravity.y;
    check("velocidad y = -g*n*dt", b->velocity.y, -g * n * kDt, 0.5f);
    check("posicion y (simplectico)", b->position.y,
          500.0f - g * kDt * kDt * n * (n + 1.0f) / 2.0f, 0.5f);
}

// 5) Rebote contra un cuerpo estatico.
//    Restitucion del contacto = min(goma 0.9, estatico 0.5) = 0.5.
//    Orden del step: la gravedad se integra ANTES de detectar, asi que la
//    velocidad de impacto es vyAntes + g*dt, y tras el rebote vale -e * eso.
void testRebote(const Config& cfg, const MaterialTable& mats)
{
    std::puts("\n5) Rebote de goma contra suelo estatico (e = min(0.9, 0.5) = 0.5)");
    World w(cfg, mats);
    w.addCircle({0.0f, 200.0f}, 50.0f, Mat::Estatico);
    const BodyId id = w.addCircle({0.0f, 0.0f}, 10.0f, Mat::Goma);
    RigidBody* ball = w.find(id);

    bool hubo = false;
    for (int i = 0; i < 300 && !hubo; ++i)
    {
        const float vyAntes = ball->velocity.y;
        w.step(kDt);
        if (!w.contacts().empty())
        {
            hubo = true;
            const float vImpacto = vyAntes + cfg.gravity.y * kDt;
            std::printf("  contacto en el paso %d, velocidad de impacto = %.4f\n", i + 1, vImpacto);
            check("velocidad tras rebote = -0.5*impacto", ball->velocity.y,
                  -0.5f * vImpacto, 0.5f);
        }
    }
    if (!hubo)
    {
        ++g_fallos;
        std::puts("  [MAL] no hubo contacto en 300 pasos");
    }
}

}  // namespace

int main(int argc, char** argv)
{
    // El orden de declaracion importa: se destruyen en orden inverso.
    const Config cfg{};
    const MaterialTable mats;

    std::puts("=== Pruebas de fisica ===");
    testCaidaLibre(cfg, mats);
    testEstatico(cfg, mats);
    testMasaNoImporta(cfg, mats);
    testVapor(cfg, mats);
    testRebote(cfg, mats);
    std::printf("\nResultado: %s (%d fallo(s))\n", g_fallos == 0 ? "TODO OK" : "HAY FALLOS", g_fallos);

    if (argc > 1 && std::string(argv[1]) == "--tests")
        return g_fallos == 0 ? 0 : 1;

    // ---- VISUAL ----
    // Orden: config, materiales, world, ventana, renderer.
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
        if (s.mat == Mat::Vapor) world.find(id)->gravityScale = -1.0f;
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
    return g_fallos == 0 ? 0 : 1;
}