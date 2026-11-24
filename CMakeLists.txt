cmake_minimum_required(VERSION 3.20)
project(PysicD LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

# SFML se instaló en modo estático (libsfml-*-s.a): hay que avisarlo ANTES de find_package
set(SFML_STATIC_LIBRARIES TRUE)
find_package(SFML 3 COMPONENTS Graphics REQUIRED)

# ---------------------------------------------------------------------------
# Librerías
# ---------------------------------------------------------------------------

# Núcleo físico y escenas: sin SFML
add_library(pysicd_core STATIC
    RigidBody.cpp
    World.cpp
    Integrator.cpp
    CollisionDetector.cpp
    CollisionResolver.cpp
    Scenes.cpp
)
target_include_directories(pysicd_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Renderer: depende de SFML y del núcleo
add_library(pysicd_render STATIC Renderer.cpp)
target_link_libraries(pysicd_render PUBLIC pysicd_core SFML::Graphics)

# ---------------------------------------------------------------------------
# Ejecutables
# ---------------------------------------------------------------------------

# Principal del PoC: loop con acumulador, entrada, escenas y limpieza
add_executable(pysicd Main3.cpp)
target_link_libraries(pysicd PRIVATE pysicd_render)

# Demos visuales de las primeras pruebas
add_executable(pysicd_caida Main.cpp)
target_link_libraries(pysicd_caida PRIVATE pysicd_render)

add_executable(pysicd_escenas Main2.cpp)
target_link_libraries(pysicd_escenas PRIVATE pysicd_render)

# Pruebas numéricas: solo núcleo, sin SFML
add_executable(pysicd_tests Test.cpp)
target_link_libraries(pysicd_tests PRIVATE pysicd_core)
add_test(NAME fisica COMMAND pysicd_tests)

# ---------------------------------------------------------------------------
# Avisos del compilador
# ---------------------------------------------------------------------------
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    foreach(t pysicd_core pysicd_render pysicd pysicd_caida pysicd_escenas pysicd_tests)
        target_compile_options(${t} PRIVATE -Wall -Wextra)
    endforeach()
endif()