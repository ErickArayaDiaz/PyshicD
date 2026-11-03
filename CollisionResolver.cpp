#include "CollisionResolver.h"
#include <algorithm>
#include "CollisionResolver.h"
#include "RigidBody.h"   // añadir: desreferencia los punteros
#include <algorithm>

namespace {
    constexpr float kPercent = 0.4f;   // fracción corregida por frame
    constexpr float kSlop    = 0.01f;  // tolerancia; calibrar a la escala de SFML
}

void CollisionResolver::resolveImpulses(const std::vector<Contact>& contacts)
{
    for (const Contact& c : contacts)
    {
        RigidBody& a = *c.a;
        RigidBody& b = *c.b;

        const float invMassSum = a.invMass + b.invMass;
        if (invMassSum == 0.0f) continue;           // dos estáticos

        const float vn = (b.velocity - a.velocity).dot(c.normal);
        if (vn > 0.0f) continue;                    // ya se separan

        const float j = -(1.0f + c.restitution) * vn / invMassSum;
        const Vector2 impulse = c.normal * j;

        a.velocity -= impulse * a.invMass;
        b.velocity += impulse * b.invMass;
    }
}

void CollisionResolver::correctPositions(const std::vector<Contact>& contacts)
{
    for (const Contact& c : contacts)
    {
        RigidBody& a = *c.a;
        RigidBody& b = *c.b;

        const float invMassSum = a.invMass + b.invMass;
        if (invMassSum == 0.0f) continue;

        const float depth = std::max(c.penetration - kSlop, 0.0f);
        const Vector2 correction = c.normal * (depth / invMassSum * kPercent);

        a.position -= correction * a.invMass;
        b.position += correction * b.invMass;
    }
}