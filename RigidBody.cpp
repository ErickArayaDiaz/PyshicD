#include "RigidBody.h"
#include <cstdio>

void RigidBody::setMassFromMaterial(const Material& m) {
    restitution = m.restitution;   // antes del return temprano
    const float newMass = m.density * area();
    if (newMass <= 0.0f) {
        std::fprintf(stderr, "Aviso: masa <= 0, cuerpo %u pasa a estatico\n", id);
        mass = 0.0f;
        invMass = 0.0f;
        return;
    }
    mass = newMass;
    invMass = 1.0f / newMass;
}