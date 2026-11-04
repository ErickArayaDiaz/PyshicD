#pragma once

#include <array>
#include <cstddef>

#include <SFML/Graphics.hpp>

#include "World.h"
#include "RigidBody.h"
#include "MaterialTable.h"


class Renderer
{
public:
    Renderer(sf::RenderWindow& window, const World& world);

    // No es const: reconfigura la forma reutilizada (m_shape) en cada cuerpo.
    void draw();

private:
    // Debe seguir el mismo orden que MaterialTable:
    // agua, vapor, piedra, goma, estático.
    static constexpr std::size_t kMaterialCount = 5;

    sf::Color colorFor(MaterialId id) const;

    sf::RenderWindow&                        m_window;
    const World&                             m_world;
    sf::CircleShape                          m_shape;
    std::array<sf::Color, kMaterialCount>    m_colors;
    sf::Color                                m_fallbackColor;
};