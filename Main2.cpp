// main.cpp de PRUEBAS (no es el main final del PoC).
// Parte 1: 5 verificaciones numericas de la fisica, sin ventana.
// Parte 2: demo visual de 10 s con dos escenas de 5 s cada una:
//   0-5 s : barrera en forma de CAJA   (4 lineas de circulos estaticos)
//   5-10 s: barrera en forma de ANILLO (circulos estaticos en circunferencia)
// Los cuerpos aparecen en posiciones, tamanos y materiales aleatorios.
// A los 10 s la ventana se cierra sola y el programa termina.
//
// Uso:  pysicd.exe            -> pruebas + ventana
//       pysicd.exe --tests    -> solo pruebas (sin ventana)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <optional>
#include <random>
#include <string>
#include <vector>

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

// ---------------------------------------------------------------------------
// Escenas de la demo visual
// ---------------------------------------------------------------------------
constexpr float kVentanaW = 800.0f;
constexpr float kVentanaH = 600.0f;
// Las paredes son circulos estaticos GRUESOS con el centro fuera de la zona visible:
// con paredes finas, un cuerpo rapido (>~1000 px/s) puede quedar ya pasado el centro del
// circulo de pared en su primer paso de contacto, y la normal lo empuja hacia afuera.
constexpr float kParedR    = 30.0f;  // radio de cada circulo de pared
constexpr float kParedPaso = 14.0f;  // separacion maxima (< 2*radio: se solapan, sin huecos)
constexpr float kCara      = 20.0f;  // distancia de la cara interior de la pared al borde
constexpr int   kCuerpos  = 15;

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

    const unsigned semilla = std::random_device{}() ^
        static_cast<unsigned>(std::chrono::steady_clock::now().time_since_epoch().count());
    std::mt19937 rng(semilla);
    std::printf("\nSemilla aleatoria: %u\n", semilla);

    const float tCambio = 5.0f;    // pasa de la caja al anillo
    const float tFin    = 10.0f;   // cierra la ventana
    const float maxFrame = 0.1f;   // tope de tiempo por frame (anti espiral)

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
    return g_fallos == 0 ? 0 : 1;
}