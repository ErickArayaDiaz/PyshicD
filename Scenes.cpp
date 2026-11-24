#include "Scenes.h"
#include "MaterialTable.h"   // Mat::Estatico
#include "RigidBody.h"       // PI
#include <cmath>
#include <algorithm>
#include "Constants.h" //PI

namespace
{
    constexpr float kParedR    = 30.f;   // radio de los círculos de pared
    constexpr float kParedPaso = 14.f;   // separación entre centros (< 2*kParedR)

    void pared(World& w, float x, float y)
    {
        w.addCircle(Vector2(x, y), kParedR, Mat::Estatico);
    }

    void linea(World& w, float ax, float ay, float bx, float by, bool incluirFin = true)
    {
        const float largo = std::hypot(bx - ax, by - ay);
        const int   n     = std::max(1, static_cast<int>(std::ceil(largo / kParedPaso)));
        const int   fin   = incluirFin ? n : n - 1;
        for (int i = 0; i <= fin; ++i)
        {
            const float t = static_cast<float>(i) / static_cast<float>(n);
            pared(w, ax + (bx - ax) * t, ay + (by - ay) * t);
        }
    }

    void arco(World& w, float cx, float cy, float radio, float angIni, float angFin)
    {
        const float span    = angFin - angIni;
        const bool  cerrado = std::abs(span) >= 2.f * PI - 1e-4f;
        const int   n       = std::max(1, static_cast<int>(std::ceil(std::abs(span) * radio / kParedPaso)));
        const int   fin     = cerrado ? n - 1 : n;
        for (int i = 0; i <= fin; ++i)
        {
            const float ang = angIni + span * static_cast<float>(i) / static_cast<float>(n);
            pared(w, cx + std::cos(ang) * radio, cy + std::sin(ang) * radio);
        }
    }

    // La CARA superior va de a a b; los centros suben kParedR por la normal.
    void rampa(World& w, float ax, float ay, float bx, float by)
    {
        const float len = std::hypot(bx - ax, by - ay);
        float nx =  (by - ay) / len;
        float ny = -(bx - ax) / len;
        if (ny > 0.f) { nx = -nx; ny = -ny; }
        const float ox = nx * kParedR, oy = ny * kParedR;
        linea(w, ax + ox, ay + oy, bx + ox, by + oy);
    }
}

const char* nombreEscena(Escena e)
{
    switch (e)
    {
    case Escena::Caja:   return "Caja";
    case Escena::Anillo: return "Anillo";
    case Escena::Cuenco: return "Cuenco";
    case Escena::Rampas: return "Rampas";
    }
    return "?";
}

void cargarEscena(World& world, Escena e, float W, float H)
{
    world.clearAll();

    const float m  = 20.f;               // cara interior a 20 px del borde
    const float cx = W * 0.5f, cy = H * 0.5f;

    switch (e)
    {
    case Escena::Caja:
    {
        const float x0 = m - kParedR, x1 = W - m + kParedR;
        const float y0 = m - kParedR, y1 = H - m + kParedR;
        linea(world, x0, y0, x1, y0, false);
        linea(world, x1, y0, x1, y1, false);
        linea(world, x1, y1, x0, y1, false);
        linea(world, x0, y1, x0, y0, false);
        break;
    }
    case Escena::Anillo:
    {
        const float cara = std::min(W, H) * 0.5f - m;
        arco(world, cx, cy, cara + kParedR, 0.f, 2.f * PI);
        break;
    }
    case Escena::Cuenco:
    {
        // El fondo de la cara queda a m px del borde inferior
        const float cara    = W * 0.4f;
        const float centroY = H - m - cara;
        arco(world, cx, centroY, cara + kParedR, 0.f, PI);
        break;
    }
    case Escena::Rampas:
        rampa(world, W * 0.05f, H * 0.25f, W * 0.65f, H * 0.38f);
        rampa(world, W * 0.95f, H * 0.52f, W * 0.35f, H * 0.65f);
        rampa(world, W * 0.05f, H * 0.79f, W * 0.65f, H * 0.92f);
        break;
    }
}