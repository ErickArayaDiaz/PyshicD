#pragma once
#include <vector>
#include "Contact.h"

class CollisionResolver {
public:
    // Solo modifica velocidades
    static void resolveImpulses(const std::vector<Contact>& contacts);
    // Solo modifica posiciones
    static void correctPositions(const std::vector<Contact>& contacts);
};