#pragma once
#include "Vector2.h"

class RigidBody;   // declaración adelantada: basta para guardar punteros

struct Contact {
    RigidBody* a;        // cuerpo A (no propietario)
    RigidBody* b;        // cuerpo B (no propietario)
    Vector2 normal;      // dirección de A hacia B
    float penetration;   // cuánto se solapan
    float restitution = 0.0f;
};