#include "CollisionDetector.h"
#include "Constants.h"
#include <algorithm>

void CollisionDetector::detect(const std::vector<std::unique_ptr<RigidBody>>& bodies,
                               std::vector<Contact>& out)
{
    for (std::size_t i = 0; i < bodies.size(); ++i)
    {
        for (std::size_t j = i + 1; j < bodies.size(); ++j)
        {
            RigidBody& a = *bodies[i];
            RigidBody& b = *bodies[j];

            if (a.invMass == 0.0f && b.invMass == 0.0f) continue;   // dos estáticos

            if (auto c = circleVsCircle(a, b))
                out.push_back(*c);
        }
    }
}

std::optional<Contact> CollisionDetector::circleVsCircle(RigidBody& a, RigidBody& b)
{
    const Vector2 delta = b.position - a.position;
    const float distSq = delta.lengthSq();
    const float radiusSum = a.radius + b.radius;

    if (distSq >= radiusSum * radiusSum)
        return std::nullopt;

    const float d = delta.length();

    Contact c;
    c.a = &a;
    c.b = &b;
    c.restitution = std::min(a.restitution, b.restitution);

    if (d < EPSILON)
    {
        c.normal = Vector2(1.0f, 0.0f);
        c.penetration = a.radius;
    }
    else
    {
        c.normal = delta / d;
        c.penetration = radiusSum - d;
    }
    return c;
}