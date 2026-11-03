#pragma once

#include <memory>
#include <vector>
#include "RigidBody.h"   

// Integrator.h
class Integrator {
public:
    Integrator() = delete;

    // Euler simpléctico, parte 1: v += F * invMass * dt.
    // EFECTO LATERAL: deja force en cero en todos los cuerpos (también estáticos).
    static void integrateVelocities(std::vector<std::unique_ptr<RigidBody>>& bodies, float dt);

    // Parte 2: p += v * dt. Ignora los estáticos.
    static void integratePositions(std::vector<std::unique_ptr<RigidBody>>& bodies, float dt);
};