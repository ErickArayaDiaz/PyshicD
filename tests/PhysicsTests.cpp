#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "Integrator.h"
#include "MaterialTable.h"
#include "RigidBody.h"
#include "World.h"

namespace {
constexpr float kDt = 1.0f / 60.0f;
constexpr int kSteps = 60;
constexpr float kGravity = 980.0f;
constexpr float kExpectedY = 498.1666667f;
constexpr float kTolerance = 0.01f;
constexpr double kStressDurationSeconds = 18.0;
constexpr std::size_t kStressBodyCount = 1024;

class TestSuite {
public:
    explicit TestSuite(std::ostream& report) : m_report(report) {}

    void check(const std::string& name, bool passed, const std::string& details = {}) {
        ++m_total;
        if (!passed)
            ++m_failed;
        const std::string result = passed ? "PASS" : "FAIL";
        m_report << "[" << result << "] " << name;
        if (!details.empty())
            m_report << " -- " << details;
        m_report << '\n';
    }

    int total() const { return m_total; }
    int failed() const { return m_failed; }

private:
    std::ostream& m_report;
    int m_total = 0;
    int m_failed = 0;
};

bool near(float actual, float expected, float tolerance = kTolerance) {
    return std::fabs(actual - expected) <= tolerance;
}

void testVector2(TestSuite& tests, std::ostream& report) {
    const Vector2 a{3.0f, 4.0f};
    const Vector2 b{-1.0f, 2.0f};
    const Vector2 sum = a + b;
    const Vector2 difference = a - b;
    const Vector2 normalized = a.normalized();
    const Vector2 zeroNormalized = Vector2{}.normalized();

    tests.check("Vector2 addition", sum.x == 2.0f && sum.y == 6.0f);
    tests.check("Vector2 subtraction", difference.x == 4.0f && difference.y == 2.0f);
    tests.check("Vector2 scalar multiply/divide",
                (a * 2.0f).x == 6.0f && (a * 2.0f).y == 8.0f &&
                (a / 2.0f).x == 1.5f && (a / 2.0f).y == 2.0f);
    tests.check("Vector2 dot product", a.dot(b) == 5.0f);
    tests.check("Vector2 length", near(a.length(), 5.0f, 1e-6f));
    tests.check("Vector2 normalization",
                near(normalized.x, 0.6f, 1e-6f) && near(normalized.y, 0.8f, 1e-6f));
    tests.check("Vector2 zero normalization",
                zeroNormalized.x == 0.0f && zeroNormalized.y == 0.0f);

    report << "\nVector2 detail: a=(3,4), b=(-1,2), sum=(" << sum.x << ',' << sum.y
           << "), difference=(" << difference.x << ',' << difference.y
           << "), dot=" << a.dot(b) << ", length(a)=" << a.length()
           << ", normalized(a)=(" << normalized.x << ',' << normalized.y << ")\n";
}

void testRigidBody(TestSuite& tests, std::ostream& report) {
    MaterialTable materials;
    RigidBody body{{0.0f, 0.0f}, 2.0f, Mat::Piedra};
    body.setMassFromMaterial(materials.get(Mat::Piedra));
    const float expectedMass = 2.5f * PI * 4.0f;
    body.applyForce({3.0f, 4.0f});
    body.clearForces();

    tests.check("RigidBody material identity",
                body.materialId == Mat::Piedra && body.material == Mat::Piedra);
    tests.check("RigidBody area and material mass",
                near(body.area(), PI * 4.0f, 1e-5f) &&
                near(body.mass, expectedMass, 1e-5f) &&
                near(body.invMass, 1.0f / expectedMass, 1e-5f));
    tests.check("RigidBody apply/clear force",
                body.force.x == 0.0f && body.force.y == 0.0f);

    RigidBody staticBody;
    staticBody.velocity = {4.0f, 5.0f};
    staticBody.setMass(0.0f);
    tests.check("RigidBody zero mass is static",
                staticBody.mass == 0.0f && staticBody.invMass == 0.0f);

    report << "RigidBody detail: material=" << materials.get(Mat::Piedra).name
           << ", density=" << materials.get(Mat::Piedra).density
           << ", radius=2, area=" << body.area() << ", mass=" << body.mass
           << ", invMass=" << body.invMass << '\n';
}

void testIntegrator(TestSuite& tests, std::ostream& report) {
    std::vector<std::unique_ptr<RigidBody>> bodies;
    auto dynamicBody = std::make_unique<RigidBody>();
    dynamicBody->setMass(2.0f);
    dynamicBody->force = {10.0f, 0.0f};
    bodies.push_back(std::move(dynamicBody));

    auto staticBody = std::make_unique<RigidBody>();
    staticBody->force = {8.0f, 9.0f};
    staticBody->velocity = {1.0f, 2.0f};
    staticBody->position = {3.0f, 4.0f};
    bodies.push_back(std::move(staticBody));

    Integrator::integrateVelocities(bodies, 1.0f);
    Integrator::integratePositions(bodies, 0.5f);

    tests.check("Integrator velocity update", bodies[0]->velocity.x == 5.0f);
    tests.check("Integrator consumes dynamic force",
                bodies[0]->force.x == 0.0f && bodies[0]->force.y == 0.0f);
    tests.check("Integrator consumes static force",
                bodies[1]->force.x == 0.0f && bodies[1]->force.y == 0.0f);
    tests.check("Integrator advances dynamic position",
                bodies[0]->position.x == 2.5f && bodies[0]->position.y == 0.0f);
    tests.check("Integrator leaves static body unchanged",
                bodies[1]->position.x == 3.0f && bodies[1]->position.y == 4.0f &&
                bodies[1]->velocity.x == 1.0f && bodies[1]->velocity.y == 2.0f);

    report << "Integrator detail: mass=2, force=(10,0), dt=1 => velocity=("
           << bodies[0]->velocity.x << ',' << bodies[0]->velocity.y
           << "), position after dt=0.5=(" << bodies[0]->position.x << ','
           << bodies[0]->position.y << "); static force cleared and state unchanged\n";
}

void runWorldFreeFall(
    TestSuite& tests,
    std::ostream& report,
    float gravityScale,
    const char* label,
    float expectedY)
{
    MaterialTable materials;
    Config config;
    config.gravity = {0.0f, kGravity};
    World world(config, materials);
    const BodyId id = world.addCircle({0.0f, 0.0f}, 1.0f, Mat::Agua);
    RigidBody* body = world.find(id);

    const bool validBody = id != INVALID_ID && body != nullptr;
    tests.check(std::string("World creates ") + label + " test body", validBody);
    if (!validBody)
        return;

    body->gravityScale = gravityScale;
    report << "\n" << label << " free-fall: steps=" << kSteps
           << ", dt=" << kDt << ", gravity=(0," << kGravity
           << "), gravityScale=" << gravityScale << '\n';
    report << "step, time, x, y, vx, vy\n";
    report << std::fixed << std::setprecision(6);

    for (int step = 1; step <= kSteps; ++step) {
        world.step(kDt);
        report << step << ", " << step * kDt << ", "
               << body->position.x << ", " << body->position.y << ", "
               << body->velocity.x << ", " << body->velocity.y << '\n';
    }

    const float actualY = body->position.y;
    const float actualVy = body->velocity.y;
    const bool passed = near(actualY, expectedY) &&
                        near(actualVy, gravityScale * kGravity, 0.02f);
    tests.check(std::string("World 60-step free-fall ") + label,
                passed,
                "final y=" + std::to_string(actualY) +
                ", expected y=" + std::to_string(expectedY) +
                ", final vy=" + std::to_string(actualVy));
}

void testWorldValidationAndRemoval(TestSuite& tests) {
    MaterialTable materials;
    Config config;
    World world(config, materials);

    const BodyId invalidRadius = world.addCircle({0.0f, 0.0f}, 0.0f, Mat::Agua);
    const BodyId invalidMaterial = world.addCircle({0.0f, 0.0f}, 1.0f, 999U);
    const BodyId valid = world.addCircle({1.0f, 2.0f}, 1.0f, Mat::Vapor);
    const RigidBody* vapor = world.find(valid);

    tests.check("World rejects invalid circle inputs",
                invalidRadius == INVALID_ID && invalidMaterial == INVALID_ID);
    tests.check("World creates vapor using MaterialTable",
                vapor != nullptr && vapor->materialId == Mat::Vapor &&
                vapor->material == Mat::Vapor &&
                near(vapor->mass, materials.get(Mat::Vapor).density * PI, 1e-5f));

    world.requestRemove(valid);
    tests.check("World removes requested body", world.find(valid) == nullptr);
    world.clearAll();
    tests.check("World clearAll leaves no bodies", world.bodies().empty());
}

void runTimedWorldStress(TestSuite& tests, std::ostream& report) {
    MaterialTable materials;
    Config config;
    config.gravity = {0.0f, kGravity};
    World world(config, materials);
    bool createdAllBodies = true;

    for (std::size_t index = 0; index < kStressBodyCount; ++index) {
        const MaterialId material = static_cast<MaterialId>(index % 4);
        const BodyId id = world.addCircle(
            {static_cast<float>(index), 0.0f}, 1.0f, material);
        RigidBody* body = world.find(id);
        if (id == INVALID_ID || body == nullptr) {
            createdAllBodies = false;
            break;
        }
        body->gravityScale = index % 2 == 0 ? 1.0f : -1.0f;
    }

    tests.check("World stress setup creates all bodies",
                createdAllBodies && world.bodies().size() == kStressBodyCount);
    if (!createdAllBodies || world.bodies().size() != kStressBodyCount)
        return;

    report << "\nTimed World stress test\n"
           << "=======================\n"
           << "Target duration: " << kStressDurationSeconds << " seconds minimum\n"
           << "Bodies: " << kStressBodyCount
           << " (alternating gravityScale +1/-1; materials cycle through MaterialTable)\n"
           << "Every completed simulation step is logged below. Position and velocity "
              "columns are sums across all bodies.\n"
           << "step, elapsed_seconds, body_count, sum_x, sum_y, sum_vx, sum_vy\n";

    const auto start = std::chrono::steady_clock::now();
    std::size_t steps = 0;
    double sumX = 0.0;
    double sumY = 0.0;
    double sumVelocityX = 0.0;
    double sumVelocityY = 0.0;
    bool allFinite = true;
    double elapsedSeconds = 0.0;

    do {
        world.step(kDt);
        ++steps;

        sumX = 0.0;
        sumY = 0.0;
        sumVelocityX = 0.0;
        sumVelocityY = 0.0;
        for (const auto& body : world.bodies()) {
            sumX += body->position.x;
            sumY += body->position.y;
            sumVelocityX += body->velocity.x;
            sumVelocityY += body->velocity.y;
            allFinite = allFinite &&
                std::isfinite(body->position.x) &&
                std::isfinite(body->position.y) &&
                std::isfinite(body->velocity.x) &&
                std::isfinite(body->velocity.y) &&
                std::isfinite(body->force.x) &&
                std::isfinite(body->force.y);
        }

        elapsedSeconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start).count();
        report << steps << ", " << std::fixed << std::setprecision(6)
               << elapsedSeconds << ", " << world.bodies().size() << ", "
               << sumX << ", " << sumY << ", "
               << sumVelocityX << ", " << sumVelocityY << '\n';
    } while (elapsedSeconds < kStressDurationSeconds);

    report.flush();
    elapsedSeconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();

    bool bodiesMatchExpected = true;
    const double dt = static_cast<double>(kDt);
    const double stepCount = static_cast<double>(steps);
    const double expectedVelocityMagnitude = kGravity * dt * stepCount;
    const double expectedPositionMagnitude =
        kGravity * dt * dt * stepCount * (stepCount + 1.0) / 2.0;
    for (std::size_t index = 0; index < world.bodies().size(); ++index) {
        const RigidBody& body = *world.bodies()[index];
        const double scale = index % 2 == 0 ? 1.0 : -1.0;
        const double expectedY = scale * expectedPositionMagnitude;
        const double expectedVelocityY = scale * expectedVelocityMagnitude;
        const double positionTolerance =
            std::max(0.1, std::fabs(expectedY) * 0.001);
        const double velocityTolerance =
            std::max(0.02, std::fabs(expectedVelocityY) * 0.001);
        bodiesMatchExpected = bodiesMatchExpected &&
            std::fabs(static_cast<double>(body.position.y) - expectedY) <= positionTolerance &&
            std::fabs(static_cast<double>(body.velocity.y) - expectedVelocityY) <= velocityTolerance &&
            body.position.x == static_cast<float>(index) &&
            body.velocity.x == 0.0f &&
            body.force.x == 0.0f &&
            body.force.y == 0.0f;
    }

    report << "Stress summary: steps=" << steps
           << ", elapsed_seconds=" << std::fixed << std::setprecision(6)
           << elapsedSeconds << ", bodies=" << world.bodies().size()
           << ", final_sum_y=" << sumY
           << ", final_sum_vy=" << sumVelocityY << '\n';
    tests.check("World timed stress runs at least 18 seconds with finite, expected states",
                elapsedSeconds >= kStressDurationSeconds &&
                steps > 0 && allFinite && bodiesMatchExpected,
                "elapsed_seconds=" + std::to_string(elapsedSeconds) +
                ", steps=" + std::to_string(steps) +
                ", bodies=" + std::to_string(world.bodies().size()) +
                ", all_finite=" + std::string(allFinite ? "yes" : "no") +
                ", expected_states=" + std::string(bodiesMatchExpected ? "yes" : "no"));
}
} // namespace

int main(int argc, char** argv) {
    const std::string reportPath = argc > 1 ? argv[1] : "tests/PhysicsTestResults.txt";
    std::ofstream report(reportPath, std::ios::out | std::ios::trunc);
    if (!report) {
        std::cerr << "ERROR: could not open report file: " << reportPath << '\n';
        return 2;
    }

    report << "Physics test run\n"
           << "================\n"
           << "Collision detector/resolver are no-collision test doubles; these tests cover "
              "Vector2, RigidBody, Integrator, World lifecycle and World gravity integration.\n\n"
           << std::fixed << std::setprecision(6);

    TestSuite tests(report);
    testVector2(tests, report);
    testRigidBody(tests, report);
    testIntegrator(tests, report);
    testWorldValidationAndRemoval(tests);
    runWorldFreeFall(tests, report, 1.0f, "positive gravity", kExpectedY);
    runWorldFreeFall(tests, report, -1.0f, "negative gravity", -kExpectedY);
    runTimedWorldStress(tests, report);

    report << "\nSUMMARY\n"
           << "=======\n"
           << "Passed: " << tests.total() - tests.failed() << '\n'
           << "Failed: " << tests.failed() << '\n'
           << "Total:  " << tests.total() << '\n';
    report.close();

    std::cout << "Test report: " << reportPath << '\n'
              << "Passed: " << tests.total() - tests.failed()
              << ", Failed: " << tests.failed()
              << ", Total: " << tests.total() << '\n';
    return tests.failed() == 0 ? 0 : 1;
}
