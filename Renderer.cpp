#include "Renderer.h"

Renderer::Renderer(sf::RenderWindow& window, const World& world)
    : m_window(window)
    , m_world(world)
    , m_shape()
    , m_colors{
          sf::Color(64, 128, 255),   // agua
          sf::Color(200, 220, 235),  // vapor
          sf::Color(130, 130, 130),  // piedra
          sf::Color(230, 90, 60),    // goma
          sf::Color(60, 60, 70)      // estático
      }
    , m_fallbackColor(255, 0, 255)   // magenta: salta a la vista si falta un color
{
}

sf::Color Renderer::colorFor(MaterialId id) const
{
    const auto index = static_cast<std::size_t>(id);
    if (index >= m_colors.size())
        return m_fallbackColor;
    return m_colors[index];
}

void Renderer::draw()
{
    // b es const RigidBody&: escribir b.position = ... no compila.
    m_world.forEachBody([this](const RigidBody& b)
    {
        // 1. Leer
        const float r = static_cast<float>(b.radius);
        const float x = static_cast<float>(b.position.x);
        const float y = static_cast<float>(b.position.y);
        const MaterialId material = b.material;

        // 2. Radio ANTES que el origen
        m_shape.setRadius(r);
        // 3. Origen = centro de la forma en coordenadas locales
        m_shape.setOrigin({r, r});
        // 4. Posición del cuerpo (Vector2 propio -> sf::Vector2f)
        m_shape.setPosition({x, y});
        // 5. Color según material
        m_shape.setFillColor(colorFor(material));

        // 6. Dibujar
        m_window.draw(m_shape);
    });
}