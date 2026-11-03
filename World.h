#pragma once
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>
#include "Vector2.h"
#include "RigidBody.h"
#include "Contact.h"
#include "Config.h"
#include "MaterialTable.h"

using BodyId = std::uint32_t;
constexpr BodyId INVALID_ID = std::numeric_limits<BodyId>::max();

struct BodyDesc {
    BodyId id;
    Vector2 position;
    float radius;
    MaterialId material;
};

class World {
public:
    World(const Config& config, const MaterialTable& materials);

    BodyId addCircle(Vector2 position, float radius, MaterialId material);
    void requestRemove(BodyId id);
    RigidBody* find(BodyId id);
    void step(float dt);
    void clearAll();

    const std::vector<std::unique_ptr<RigidBody>>& bodies() const { return m_bodies; }
    std::vector<std::unique_ptr<RigidBody>>& bodies() { return m_bodies; }
    const std::vector<Contact>& contacts() const { return m_contacts; }

private:
    void applyAdd(const BodyDesc& desc);
    void applyRemove(BodyId id);
    void applyGravity();   // provisional: se reemplaza por ForceGenerators en la fase 5

    std::vector<std::unique_ptr<RigidBody>> m_bodies;
    std::vector<Contact> m_contacts;
    std::vector<BodyDesc> m_addQueue;
    std::vector<BodyId> m_removeQueue;
    BodyId m_nextId = 0;
    bool m_stepping = false;
    const Config& m_config;
    const MaterialTable& m_materials;
};