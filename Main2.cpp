// Demo visual 2: 10 s con dos escenas de 5 s.
//   0-5 s : barrera en forma de CAJA   (4 lineas de circulos estaticos)
//   5-10 s: barrera en forma de ANILLO (circulos estaticos en circunferencia)
// Los cuerpos aparecen en posiciones, tamanos y materiales aleatorios.
// A los 10 s la ventana se cierra sola.
// Las pruebas numericas estan en Test.cpp.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <optional>
#include <random>
#include <vector>

#include <SFML/Graphics.hpp>

#include "Config.h"
#include "Constants.h"
#include "MaterialTable.h"
#include "Renderer.h"
#include "World.h"

namespace {

constexpr float kDt = 1.0f / 60.0f;

constexpr float kVentanaW = 800.0f;
constexpr float kVentanaH = 600.0f;
// Paredes de circulos estaticos GRUESOS con el centro fuera de la zona visible:
// con paredes finas, un cuerpo rapido puede quedar ya pasado el centro del
// circulo de pared en su primer paso de contacto, y la normal lo empuja hacia afuera.
constexpr float kParedR    = 30.0f;  // radio de cada circulo de pared
constexpr float kParedPaso = 14.0f;  // separacion maxima (< 2*radio: sin huecos)
constexpr float kCara      = 20.0f;  // distancia de la cara interior al borde
constexpr int   kCuerpos   = 15;

enum class Escena { Caja, Anillo };

// Fila de circulos estaticos de a hasta b.
void lineaPared(World& w, Vector2 a, Vector2 b)
{
    const Vector2 d = b - a;
    const int n = static_cast<int>(std::ceil(d.length() / kParedPaso));
    for (int i = 0; i <= n; ++i)
        w.addCircle(a + d * (static_cast<float>(i) / static_cast<float>(n)), kParedR, Mat::Estatico);
}

void paredCaja(World& w)
{
    // centros = cara interior desplazada hacia afuera un radio
    const float x0 = kCara - kParedR, x1 = kVentanaW - kCara + kParedR;
    const float y0 = kCara - kParedR, y1 = kVentanaH - kCara + kParedR;
    lineaPared(w, {x0, y0}, {x1, y0});   // techo
    lineaPared(w, {x0, y1}, {x1, y1});   // suelo
    lineaPared(w, {x0, y0}, {x0, y1});   // izquierda
    lineaPared(w, {x1, y0}, {x1, y1});   // derecha
}

void paredAnillo(World& w, Vector2 centro, float radio)
{
    const int n = static_cast<int>(std::ceil(2.0f * PI * radio / kParedPaso));
    for (int i = 0; i < n; ++i)
    {
        const float ang = 2.0f * PI * static_cast<float>(i) / static_cast<float>(n);
        w.addCircle(centro + Vector2(std::cos(ang), std::sin(ang)) * radio, kParedR, Mat::Estatico);
    }
}

// Crea 'cantidad' cuerpos en posiciones aleatorias (sin solaparse al nacer),
// con radio, material y velocidad inicial aleatorios.
template <class Muestra>
void cuerposAleatorios(World& w, std::mt19937& rng, int cantidad, Muestra muestra)
{
    struct Pos { float x, y, r; };
    std::vector<Pos> puestos;
    std::uniform_real_distribution<float> dRadio(10.0f, 25.0f);
    std::uniform_real_distribution<float> dVel(-150.0f, 150.0f);
    std::uniform_int_distribution<int> dMat(0, 3);
    const MaterialId materiales[] = {Mat::Agua, Mat::Piedra, Mat::Goma, Mat::Vapor};

    for (int i = 0; i < cantidad; ++i)
    {
        for (int intento = 0; intento < 200; ++intento)
        {
            const float r = dRadio(rng);
            const Vector2 p = muestra(rng);

            bool libre = true;
            for (const Pos& o : puestos)
                if ((Vector2(o.x, o.y) - p).length() < o.r + r + 2.0f) { libre = false; break; }
            if (!libre) continue;

            puestos.push_back({p.x, p.y, r});
            const MaterialId m = materiales[dMat(rng)];
            RigidBody* b = w.find(w.addCircle(p, r, m));
            if (b)
            {
                b->velocity = {dVel(rng), dVel(rng)};
                if (m == Mat::Vapor) b->gravityScale = -1.0f;   // el vapor sube
            }
            break;
        }
    }
}

void cargarEscena(World& w, std::mt19937& rng, Escena e)
{
    w.clearAll();
    if (e == Escena::Caja)
    {
        paredCaja(w);
        cuerposAleatorios(w, rng, kCuerpos, [](std::mt19937& g) {
            std::uniform_real_distribution<float> dx(60.0f, 740.0f), dy(60.0f, 350.0f);
            return Vector2(dx(g), dy(g));
        });
    }
    else
    {
        const Vector2 centro(kVentanaW / 2.0f, kVentanaH / 2.0f);
        paredAnillo(w, centro, 275.0f + kParedR);   // cara interior en radio 275
        cuerposAleatorios(w, rng, kCuerpos, [centro](std::mt19937& g) {
            std::uniform_real_distribution<float> da(0.0f, 2.0f * PI), du(0.0f, 1.0f);
            const float ang = da(g);
            const float rr = 210.0f * std::sqrt(du(g));   // uniforme en el disco
            return centro + Vector2(std::cos(ang), std::sin(ang)) * rr;
        });
    }
}

}  // namespace

int main()
{
    // El orden de declaracion importa: se destruyen en orden inverso.
    const Config cfg{};
    const MaterialTable mats;
    World world(cfg, mats);
    sf::RenderWindow window(sf::VideoMode({800, 600}), "PysicD - caja y anillo");
    window.setFramerateLimit(60);
    Renderer renderer(window, world);

    const unsigned semilla = std::random_device{}() ^
        static_cast<unsigned>(std::chrono::steady_clock::now().time_since_epoch().count());
    std::mt19937 rng(semilla);
    std::printf("Semilla aleatoria: %u\n", semilla);

    const float tCambio  = 5.0f;    // pasa de la caja al anillo
    const float tFin     = 10.0f;   // cierra la ventana
    const float maxFrame = 0.1f;    // tope de tiempo por frame (anti espiral)

    std::puts("Escena 1/2: caja (0-5 s)");
    cargarEscena(world, rng, Escena::Caja);
    bool enSegunda = false;

    float acc = 0.0f;
    sf::Clock reloj;    // tiempo total de la demo
    sf::Clock frame;    // tiempo entre frames
    reloj.restart();
    frame.restart();

    while (window.isOpen())
    {
        while (const std::optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
                window.close();
        }
        if (!window.isOpen()) break;

        const float t = reloj.getElapsedTime().asSeconds();
        if (t >= tFin)
        {
            window.close();
            break;
        }
        if (!enSegunda && t >= tCambio)
        {
            std::puts("Escena 2/2: anillo (5-10 s)");
            cargarEscena(world, rng, Escena::Anillo);
            enSegunda = true;
            acc = 0.0f;
        }

        acc += std::min(frame.restart().asSeconds(), maxFrame);
        while (acc >= kDt)
        {
            world.step(kDt);
            acc -= kDt;
        }

        window.clear(sf::Color(25, 25, 30));
        renderer.draw();
        window.display();
    }

    std::puts("Fin de la demo.");
    return 0;
}