#include "Integrator.h"

void Integrator::integrateVelocities(std::vector<std::unique_ptr<RigidBody>>& bodies, float dt) {
    for (auto& b : bodies) {
        if (b->invMass != 0.0f)
            b->velocity += b->force * b->invMass * dt; // a = F * invMass
        b->clearForces();                              // la fuerza se consume cada paso
    }
}

void Integrator::integratePositions(std::vector<std::unique_ptr<RigidBody>>& bodies, float dt) {
    for (auto& b : bodies) {
        if (b->invMass == 0.0f) continue;          // estático: no se toca

        b->position += b->velocity * dt;           // usa la velocidad ya actualizada
    }
}