#include <algorithm>
#include <cmath>
#include <array>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "CollisionDetector.h"
#include "CollisionResolver.h"
#include "Integrator.h"
#include "MaterialTable.h"
#include "RigidBody.h"
#include "World.h"

namespace {
constexpr float kTolerance = 1.0e-3f;
constexpr float kDt = 1.0f / 120.0f;
constexpr float kGravity = 9.81f;
constexpr int kStressSteps = 1200;
constexpr std::size_t kDynamicBodyCount = 64;
constexpr std::size_t kBaseBodyCount = 24;
constexpr std::array<MaterialId, 4> kDynamicMaterials{
    Mat::Agua, Mat::Vapor, Mat::Piedra, Mat::Goma
};

std::size_t dynamicMaterialIndex(MaterialId material) {
    for (std::size_t i = 0; i < kDynamicMaterials.size(); ++i)
        if (kDynamicMaterials[i] == material)
            return i;
    return kDynamicMaterials.size();
}

class TestSuite {
public:
    explicit TestSuite(std::ostream& report) : m_report(report) {}

    void check(const std::string& name, bool passed, const std::string& details = {}) {
        ++m_total;
        if (!passed) {
            ++m_failed;
            if (m_firstFailure.empty())
                m_firstFailure = name + (details.empty() ? "" : ": " + details);
        }
        m_report << '[' << (passed ? "PASS" : "FAIL") << "] " << name;
        if (!details.empty())
            m_report << " -- " << details;
        m_report << '\n';
    }

    int total() const { return m_total; }
    int failed() const { return m_failed; }
    const std::string& firstFailure() const { return m_firstFailure; }

private:
    std::ostream& m_report;
    int m_total = 0;
    int m_failed = 0;
    std::string m_firstFailure;
};

bool near(float actual, float expected, float tolerance = kTolerance) {
    return std::fabs(actual - expected) <= tolerance;
}

void testVector2(TestSuite& tests) {
    const Vector2 a{3.0f, 4.0f};
    const Vector2 b{-1.0f, 2.0f};
    const Vector2 normalized = a.normalized();
    tests.check("Vector2 arithmetic and dot product",
                (a + b).x == 2.0f && (a + b).y == 6.0f &&
                (a - b).x == 4.0f && (a - b).y == 2.0f &&
                a.dot(b) == 5.0f);
    tests.check("Vector2 length and normalization",
                near(a.length(), 5.0f) &&
                near(normalized.x, 0.6f) && near(normalized.y, 0.8f) &&
                Vector2{}.normalized().lengthSq() == 0.0f);
}

void testRigidBodyAndIntegrator(TestSuite& tests, std::ostream& report) {
    MaterialTable materials;
    RigidBody body{{0.0f, 0.0f}, 2.0f, Mat::Goma};
    body.setMassFromMaterial(materials.get(Mat::Goma));
    body.applyForce({10.0f, 0.0f});
    std::vector<std::unique_ptr<RigidBody>> bodies;
    bodies.push_back(std::make_unique<RigidBody>(body));

    RigidBody fixed;
    fixed.position = {2.0f, 3.0f};
    fixed.velocity = {4.0f, 5.0f};
    fixed.applyForce({9.0f, 9.0f});
    fixed.setMassFromMaterial(materials.get(Mat::Estatico));
    bodies.push_back(std::make_unique<RigidBody>(fixed));

    Integrator::integrateVelocities(bodies, 0.5f);
    Integrator::integratePositions(bodies, 0.5f);
    const RigidBody& dynamic = *bodies[0];
    const RigidBody& staticBody = *bodies[1];

    tests.check("RigidBody material mass, restitution, and force clearing",
                near(dynamic.mass, 1.2f * PI * 4.0f) &&
                near(dynamic.restitution, 0.9f) &&
                dynamic.force.lengthSq() == 0.0f);
    tests.check("Integrator updates dynamic body and preserves static body",
                near(dynamic.velocity.x, 5.0f / dynamic.mass) &&
                near(dynamic.position.x, dynamic.velocity.x * 0.5f) &&
                staticBody.invMass == 0.0f &&
                staticBody.position.x == 2.0f && staticBody.position.y == 3.0f &&
                staticBody.velocity.x == 4.0f && staticBody.velocity.y == 5.0f &&
                staticBody.force.lengthSq() == 0.0f);
    report << "Integrator check: dynamic vx=" << dynamic.velocity.x
           << ", dynamic x=" << dynamic.position.x
           << ", static position=(" << staticBody.position.x << ','
           << staticBody.position.y << ")\n";
}

void testCollisionDetectorAndResolver(TestSuite& tests, std::ostream& report) {
    RigidBody base{{0.0f, 0.0f}, 2.0f, Mat::Estatico};
    base.setMass(0.0f);
    base.restitution = 0.5f;
    RigidBody ball{{0.0f, 3.5f}, 2.0f, Mat::Goma};
    ball.setMass(2.0f);
    ball.restitution = 0.9f;

    std::vector<std::unique_ptr<RigidBody>> bodies;
    bodies.push_back(std::make_unique<RigidBody>(base));
    bodies.push_back(std::make_unique<RigidBody>(ball));
    std::vector<Contact> contacts;
    CollisionDetector::detect(bodies, contacts);
    const bool detected = contacts.size() == 1 &&
        near(contacts[0].penetration, 0.5f) &&
        near(contacts[0].normal.x, 0.0f) &&
        near(contacts[0].normal.y, 1.0f) &&
        near(contacts[0].restitution, 0.5f);
    tests.check("CollisionDetector reports overlap, normal, penetration, and restitution",
                detected, "expected one contact for static base and dynamic circle");
    if (!detected)
        return;

    bodies[1]->velocity = {0.0f, -3.0f};
    CollisionResolver::resolveImpulses(contacts);
    const bool impulseApplied = near(bodies[1]->velocity.y, 1.5f) &&
                                bodies[0]->velocity.lengthSq() == 0.0f;
    tests.check("CollisionResolver impulse bounces circle without moving static base",
                impulseApplied, "expected rebound velocity y=1.5");

    const Vector2 basePosition = bodies[0]->position;
    CollisionResolver::correctPositions(contacts);
    tests.check("CollisionResolver corrects penetration and leaves base fixed",
                bodies[0]->position.x == basePosition.x &&
                bodies[0]->position.y == basePosition.y &&
                bodies[1]->position.y > 3.5f);

    bodies[1]->position = {0.0f, 4.0f};
    contacts.clear();
    CollisionDetector::detect(bodies, contacts);
    tests.check("CollisionDetector excludes touching circles", contacts.empty());

    report << "Direct collision check: rebound vy=" << bodies[1]->velocity.y
           << ", static base stayed at y=" << bodies[0]->position.y << '\n';
}

void testWorldLifecycle(TestSuite& tests) {
    MaterialTable materials;
    Config config;
    World world(config, materials);
    const BodyId badRadius = world.addCircle({0.0f, 0.0f}, 0.0f, Mat::Agua);
    const BodyId badMaterial = world.addCircle({0.0f, 0.0f}, 1.0f, 999U);
    const BodyId body = world.addCircle({1.0f, 2.0f}, 1.0f, Mat::Agua);
    tests.check("World validates input and creates valid body",
                badRadius == INVALID_ID && badMaterial == INVALID_ID &&
                world.find(body) != nullptr);
    world.requestRemove(body);
    tests.check("World removes requested body", world.find(body) == nullptr);
}

void testWorldGravityAndCollisionStress(TestSuite& tests, std::ostream& report) {
    MaterialTable materials;
    Config config;
    config.gravity = {0.0f, kGravity};
    World world(config, materials);
    std::vector<Vector2> basePositions;
    for (std::size_t i = 0; i < kBaseBodyCount; ++i) {
        const float x = -34.5f + static_cast<float>(i) * 3.0f;
        const BodyId id = world.addCircle({x, 100.0f}, 2.0f, Mat::Estatico);
        if (id == INVALID_ID || world.find(id) == nullptr) {
            tests.check("World stress setup creates static base", false,
                        "failed while creating base circle " + std::to_string(i));
            return;
        }
        basePositions.push_back(world.find(id)->position);
    }

    std::vector<BodyId> dynamicIds;
    std::vector<Vector2> initialPositions;
    for (std::size_t i = 0; i < kDynamicBodyCount; ++i) {
        const std::size_t column = i % 16;
        const std::size_t row = i / 16;
        const float x = -9.0f + static_cast<float>(column) * 1.2f;
        const float y = 80.0f - static_cast<float>(row) * 1.3f;
        const MaterialId material = kDynamicMaterials[i % kDynamicMaterials.size()];
        const BodyId id = world.addCircle({x, y}, 0.45f, material);
        RigidBody* body = world.find(id);
        if (id == INVALID_ID || body == nullptr) {
            tests.check("World stress setup creates dynamic bodies", false,
                        "failed while creating dynamic body " + std::to_string(i));
            return;
        }
        body->velocity.x = (column % 2 == 0) ? 0.15f : -0.15f;
        dynamicIds.push_back(id);
        initialPositions.push_back(body->position);
    }

    tests.check("World stress setup contains moving bodies and static base",
                world.bodies().size() == kBaseBodyCount + kDynamicBodyCount &&
                dynamicIds.size() == kDynamicBodyCount &&
                basePositions.size() == kBaseBodyCount);

    std::array<std::size_t, kDynamicMaterials.size()> materialBodyCounts{};
    std::array<std::size_t, kDynamicMaterials.size()> materialBaseContacts{};
    std::array<std::size_t, kDynamicMaterials.size()> materialBodiesHit{};
    std::array<float, kDynamicMaterials.size()> materialMaxSpeed{};
    std::size_t totalContacts = 0;
    std::size_t staticBaseContacts = 0;
    std::size_t firstHitStep = 0;
    std::size_t completedSteps = 0;
    BodyId firstHitBody = INVALID_ID;
    std::string failureLocation;
    bool allFinite = true;
    bool dynamicBodiesMoved = false;
    bool gravityAffectedBodies = false;
    bool staticBaseStayedFixed = true;
    std::vector<Vector2> previousPositions(world.bodies().size());
    std::vector<bool> materialBodyHitBase(world.bodies().size(), false);
    for (const auto& body : world.bodies()) {
        previousPositions[body->id] = body->position;
        const std::size_t materialIndex = dynamicMaterialIndex(body->material);
        if (materialIndex < kDynamicMaterials.size())
            ++materialBodyCounts[materialIndex];
    }

    report << "\nFRAME_DATA_BEGIN\n"
           << "Each row is one body after one completed world step. "
              "Direction is normalized velocity; angle is degrees from +X. "
              "Stationary bodies have direction (0,0), angle 0.\n"
           << "step,time_seconds,body_id,role,material,x,y,move_dx,move_dy,"
              "vx,vy,speed,direction_x,direction_y,direction_angle_deg,"
              "contacts_this_frame,static_base_contacts_this_frame\n";

    for (int step = 1; step <= kStressSteps; ++step) {
        world.step(kDt);
        completedSteps = static_cast<std::size_t>(step);
        const std::vector<Contact>& contacts = world.contacts();
        std::vector<std::size_t> frameContacts(world.bodies().size(), 0);
        std::vector<std::size_t> frameBaseContacts(world.bodies().size(), 0);
        totalContacts += contacts.size();
        for (const Contact& contact : contacts) {
            const RigidBody* a = contact.a;
            const RigidBody* b = contact.b;
            if (a->id < frameContacts.size())
                ++frameContacts[a->id];
            if (b->id < frameContacts.size())
                ++frameContacts[b->id];
            if (a->invMass == 0.0f || b->invMass == 0.0f) {
                ++staticBaseContacts;
                const RigidBody* dynamicBody = a->invMass == 0.0f ? b : a;
                if (dynamicBody->id < frameBaseContacts.size())
                    ++frameBaseContacts[dynamicBody->id];
                const std::size_t materialIndex =
                    dynamicMaterialIndex(dynamicBody->material);
                if (materialIndex < kDynamicMaterials.size()) {
                    ++materialBaseContacts[materialIndex];
                    if (!materialBodyHitBase[dynamicBody->id]) {
                        materialBodyHitBase[dynamicBody->id] = true;
                        ++materialBodiesHit[materialIndex];
                    }
                }
                if (firstHitStep == 0) {
                    firstHitStep = static_cast<std::size_t>(step);
                    firstHitBody = dynamicBody->id;
                }
            }
        }

        for (const auto& bodyOwner : world.bodies()) {
            const RigidBody* body = bodyOwner.get();
            const bool finite = std::isfinite(body->position.x) &&
                std::isfinite(body->position.y) &&
                std::isfinite(body->velocity.x) &&
                std::isfinite(body->velocity.y) &&
                std::isfinite(body->force.x) &&
                std::isfinite(body->force.y);
            if (!finite) {
                std::ostringstream where;
                where << "step=" << step << ", body id=" << body->id
                      << ", position=(" << body->position.x << ','
                      << body->position.y << "), velocity=("
                      << body->velocity.x << ',' << body->velocity.y << ')';
                failureLocation = where.str();
                allFinite = false;
                break;
            }
            const Vector2 movement =
                body->position - previousPositions[body->id];
            const Vector2 direction = body->velocity.normalized();
            const float speed = body->velocity.length();
            const float angleDegrees =
                std::atan2(direction.y, direction.x) * 180.0f / PI;
            const bool isStatic = body->invMass == 0.0f;
            const std::size_t materialIndex =
                dynamicMaterialIndex(body->material);
            if (!isStatic && materialIndex < kDynamicMaterials.size()) {
                if (movement.lengthSq() > 0.0f)
                    dynamicBodiesMoved = true;
                if (body->velocity.y > 0.1f)
                    gravityAffectedBodies = true;
                materialMaxSpeed[materialIndex] =
                    std::max(materialMaxSpeed[materialIndex], speed);
            }
            if (!isStatic &&
                (body->position.x != initialPositions[body->id - kBaseBodyCount].x ||
                 body->position.y != initialPositions[body->id - kBaseBodyCount].y))
                dynamicBodiesMoved = true;

            report << step << ',' << step * kDt << ',' << body->id << ','
                   << (isStatic ? "static" : "dynamic") << ','
                   << materials.get(body->material).name << ','
                   << body->position.x << ',' << body->position.y << ','
                   << movement.x << ',' << movement.y << ','
                   << body->velocity.x << ',' << body->velocity.y << ','
                   << speed << ',' << direction.x << ',' << direction.y << ','
                   << angleDegrees << ',' << frameContacts[body->id] << ','
                   << frameBaseContacts[body->id] << '\n';
            previousPositions[body->id] = body->position;
        }

        for (std::size_t i = 0; i < basePositions.size(); ++i) {
            const RigidBody* base = world.find(static_cast<BodyId>(i));
            if (base == nullptr || base->position.x != basePositions[i].x ||
                base->position.y != basePositions[i].y) {
                staticBaseStayedFixed = false;
                if (failureLocation.empty()) {
                    std::ostringstream where;
                    where << "step=" << step << ", static base id=" << i
                          << ", position changed";
                    failureLocation = where.str();
                }
                break;
            }
        }
        if (!allFinite || !staticBaseStayedFixed)
            break;
    }

    report << "FRAME_DATA_END\n";

    const RigidBody* hitBody = firstHitBody == INVALID_ID
        ? nullptr : world.find(firstHitBody);
    std::ostringstream stressDetails;
    stressDetails << "completed_steps=" << completedSteps
                  << ", dynamic_bodies=" << dynamicIds.size()
                  << ", static_base_circles=" << basePositions.size()
                  << ", contacts=" << totalContacts
                  << ", static_base_contacts=" << staticBaseContacts
                  << ", gravity_affected=" << (gravityAffectedBodies ? "yes" : "no")
                  << ", first_base_hit_step=" << firstHitStep;
    if (hitBody != nullptr)
        stressDetails << ", first_hit_body=" << firstHitBody
                      << ", hit_body_final_y=" << hitBody->position.y;
    if (!failureLocation.empty())
        stressDetails << ", failure_location={" << failureLocation << '}';

    report << "\nMATERIAL_RESULTS\n"
           << "material,density,restitution,bodies,base_contact_events,"
              "bodies_that_hit_base,max_speed,final_body_states\n";
    bool everyMaterialHitBase = true;
    for (std::size_t i = 0; i < kDynamicMaterials.size(); ++i) {
        const MaterialId materialId = kDynamicMaterials[i];
        const Material& material = materials.get(materialId);
        everyMaterialHitBase = everyMaterialHitBase &&
            materialBodiesHit[i] > 0;
        report << material.name << ',' << material.density << ','
               << material.restitution << ',' << materialBodyCounts[i] << ','
               << materialBaseContacts[i] << ',' << materialBodiesHit[i] << ','
               << materialMaxSpeed[i] << ",\n";
        for (BodyId id : dynamicIds) {
            const RigidBody* body = world.find(id);
            if (body == nullptr || body->material != materialId)
                continue;
            report << "  body=" << id << " start=("
                   << initialPositions[id - kBaseBodyCount].x << ','
                   << initialPositions[id - kBaseBodyCount].y << ") final=("
                   << body->position.x << ',' << body->position.y
                   << ") velocity=(" << body->velocity.x << ','
                   << body->velocity.y << ") speed=" << body->velocity.length()
                   << " direction=(" << body->velocity.normalized().x << ','
                   << body->velocity.normalized().y << ")"
                   << " base_contacts=" << (materialBodyHitBase[id] ? "yes" : "no")
                   << '\n';
        }
    }

    tests.check("World gravity and collision stress remains finite and hits static base",
                allFinite && staticBaseStayedFixed && dynamicBodiesMoved &&
                gravityAffectedBodies && staticBaseContacts > 0 &&
                everyMaterialHitBase,
                stressDetails.str());
    report << "Stress details: " << stressDetails.str() << '\n';
    report << "Stress outcome: " << (allFinite && staticBaseStayedFixed &&
        dynamicBodiesMoved && gravityAffectedBodies &&
        staticBaseContacts > 0 && everyMaterialHitBase ? "PASS" : "FAIL") << '\n';
}
} // namespace

int main(int argc, char** argv) {
    const std::string reportPath = argc > 1 ? argv[1] : "test2/PhysicsStressResults.txt";
    std::ofstream report(reportPath, std::ios::out | std::ios::trunc);
    if (!report) {
        std::cerr << "ERROR: could not open report file: " << reportPath << '\n';
        return 2;
    }

    report << "Real physics stress test report\n"
           << "===============================\n"
           << "Uses the production collision detector and resolver; no test doubles.\n"
           << "Simulation: 64 dynamic circles cycling Agua, Vapor, Piedra, and Goma; "
              "24 static Estatico base circles, "
              "1200 steps at 1/120 s, gravity=(0,9.81).\n\n"
           << std::fixed << std::setprecision(5);

    TestSuite tests(report);
    testVector2(tests);
    testRigidBodyAndIntegrator(tests, report);
    testCollisionDetectorAndResolver(tests, report);
    testWorldLifecycle(tests);
    testWorldGravityAndCollisionStress(tests, report);

    report << "\nSUMMARY\n=======\n"
           << "Passed: " << tests.total() - tests.failed() << '\n'
           << "Failed: " << tests.failed() << '\n'
           << "Total:  " << tests.total() << '\n';
    if (!tests.firstFailure().empty())
        report << "First failure: " << tests.firstFailure() << '\n';
    report.close();

    std::cout << "Test report: " << reportPath << '\n'
              << "Passed: " << tests.total() - tests.failed()
              << ", Failed: " << tests.failed()
              << ", Total: " << tests.total() << '\n';
    if (!tests.firstFailure().empty())
        std::cout << "First failure: " << tests.firstFailure() << '\n';
    return tests.failed() == 0 ? 0 : 1;
}
