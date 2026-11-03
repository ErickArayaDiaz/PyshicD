#include "World.h"
#include <algorithm>
#include "Integrator.h"
#include "CollisionDetector.h"
#include "CollisionResolver.h"

World::World(const Config& config, const MaterialTable& materials)
    : m_config(config), m_materials(materials) {}

BodyId World::addCircle(Vector2 position, float radius, MaterialId material) {
    if (radius <= 0.0f || !m_materials.has(material))
        return INVALID_ID;

    BodyDesc desc{ m_nextId++, position, radius, material };

    if (m_stepping) m_addQueue.push_back(desc);
    else            applyAdd(desc);
    return desc.id;
}

void World::requestRemove(BodyId id) {
    if (m_stepping) m_removeQueue.push_back(id);
    else            applyRemove(id);
}

RigidBody* World::find(BodyId id) {
    for (auto& b : m_bodies)
        if (b->id == id) return b.get();
    return nullptr;
}

void World::applyAdd(const BodyDesc& desc) {
    auto body = std::make_unique<RigidBody>();
    body->id = desc.id;
    body->position = desc.position;
    body->velocity = Vector2{0.0f, 0.0f};
    body->radius = desc.radius;
    body->setMaterial(desc.material);
    body->setMassFromMaterial(m_materials.get(desc.material)); // función única: masa e invMass
    m_bodies.push_back(std::move(body));
}

void World::applyRemove(BodyId id) {
    auto it = std::find_if(m_bodies.begin(), m_bodies.end(),
                           [id](const auto& b) { return b->id == id; });
    if (it == m_bodies.end()) return;   // ya no existe: se ignora

    m_contacts.clear();                 // primero se retiran las referencias
    m_bodies.erase(it);                 // luego se destruye el cuerpo
}

void World::clearAll() {
    m_contacts.clear();
    m_addQueue.clear();
    m_removeQueue.clear();
    m_bodies.clear();                   // m_nextId NO se reinicia
}

void World::applyGravity() {
    for (auto& b : m_bodies) {
        if (b->invMass == 0.0f) continue;
        b->force += m_config.gravity * (b->gravityScale * b->mass);
    }
}

void World::step(float dt) {
    m_stepping = true;

    m_contacts.clear();                                         // 1
    applyGravity();                                             // 2
    Integrator::integrateVelocities(m_bodies, dt);              // 3 (pone fuerzas a 0)
    CollisionDetector::detect(m_bodies, m_contacts);            // 4
    CollisionResolver::resolveImpulses(m_contacts);             // 5
    Integrator::integratePositions(m_bodies, dt);               // 6
    CollisionResolver::correctPositions(m_contacts);            // 7
    // 8: sistemas post-física (fase 5)

    m_stepping = false;

    for (const auto& d : m_addQueue) applyAdd(d);               // 9: altas primero
    for (BodyId id : m_removeQueue) applyRemove(id);            // 9: bajas después
    m_addQueue.clear();
    m_removeQueue.clear();
}