#pragma once
#include <cstdint>
#include <vector>

using MaterialId = std::uint32_t;

struct Material {
    const char* name;
    float density;       // masa por unidad de área; 0 = el cuerpo queda estático
    float restitution;   // 0..1, lo usará el resolver; por ahora no estorba
};

class MaterialTable {
public:
    MaterialTable() : m_materials{
        // nombre,   densidad, restitución
        {"agua",     1.0f,     0.1f},
        {"vapor",    0.001f,   0.0f},
        {"piedra",   2.5f,     0.4f},
        {"goma",     1.2f,     0.9f},
        {"estatico", 0.0f,     0.5f},  
    } {}

    bool has(MaterialId id) const { return id < m_materials.size(); }
    const Material& get(MaterialId id) const { return m_materials[id]; }

private:
    std::vector<Material> m_materials;
};

// Nombres para no usar números sueltos. Deben seguir el orden de la tabla.
namespace Mat {
    inline constexpr MaterialId Agua   = 0;
    inline constexpr MaterialId Vapor  = 1;
    inline constexpr MaterialId Piedra = 2;
    inline constexpr MaterialId Goma   = 3;
    inline constexpr MaterialId Estatico = 4;
}