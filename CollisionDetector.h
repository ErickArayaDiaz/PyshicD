#pragma once
#include <memory>
#include <optional>
#include <vector>
#include "Contact.h"
#include "RigidBody.h"

class CollisionDetector {
public:
    CollisionDetector() = delete;

    // Llena 'out' con un Contact por cada par de cuerpos solapados.
    static void detect(const std::vector<std::unique_ptr<RigidBody>>& bodies,
                       std::vector<Contact>& out);

    static std::optional<Contact> circleVsCircle(RigidBody& a, RigidBody& b);
};