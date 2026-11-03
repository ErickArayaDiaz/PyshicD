#pragma once

#include <memory>
#include <vector>

#include "Contact.h"
#include "RigidBody.h"

class CollisionDetector {
public:
    static void detect(
        const std::vector<std::unique_ptr<RigidBody>>&,
        std::vector<Contact>& contacts)
    {
        contacts.clear();
    }
};
