#pragma once
#include <cstddef>
#include "Vector2.h"

// Paquete de datos: el detector lo produce, el resolver lo consume.
// Convención: la normal apunta de A hacia B.
struct Manifold
{
    std::size_t indexA;
    std::size_t indexB;
    Vector2 normal;
    float penetration;
};