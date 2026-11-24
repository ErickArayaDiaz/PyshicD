#pragma once
#include <cstdint>
#include <cstdio>
#include <iostream>
#include "Vector2.h"
#include "MaterialTable.h"   // Material y MaterialId
#include "Constants.h" // PI

struct RigidBody {
    // El World asigna id, posición, radio y material
    std::uint32_t id = 0;
    MaterialId material = 0;
    Vector2 position;
    Vector2 velocity;
    Vector2 force;
    float radius = 0.0f;
    float mass = 0.0f;       // 0 = estático
    float invMass = 0.0f;    // 0 = estático
    float gravityScale = 1.0f;  // negativo: sube (vapor)
    float restitution = 0.0f; // copia del material

    constexpr RigidBody() = default;
    constexpr RigidBody(Vector2 pos, float rad, MaterialId mat = 0)
    : material(mat), position(pos), radius(rad) {}

    void setMaterial(MaterialId mat) { material = mat; }

    float area() const { return PI * radius * radius; }

    void applyForce(Vector2 f) { force += f; }

    void clearForces() { force = {0.0f, 0.0f}; }

    void setMass(float newMass) {
        if (newMass <= 0.0f) {
            std::cerr << "Aviso: masa <= 0, cuerpo " << id << " pasa a estatico\n";
            mass = 0.0f;
            invMass = 0.0f;
            return;
        }
        mass = newMass;
        invMass = 1.0f / newMass;
    }

    void setMassFromMaterial(const Material& m);   // en RigidBody.cpp
};