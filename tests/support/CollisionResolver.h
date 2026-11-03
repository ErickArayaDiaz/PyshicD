#pragma once

#include <vector>

#include "Contact.h"

class CollisionResolver {
public:
    static void resolveImpulses(std::vector<Contact>&) {}
    static void correctPositions(std::vector<Contact>&) {}
};
