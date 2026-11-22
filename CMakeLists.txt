cmake_minimum_required(VERSION 3.20)
project(PysicD LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# SFML se instaló en modo estático (libsfml-*-s.a): hay que avisarlo ANTES de find_package
set(SFML_STATIC_LIBRARIES TRUE)
find_package(SFML 3 COMPONENTS Graphics REQUIRED)

# --- Núcleo físico: sin SFML ---
add_library(pysicd_core STATIC
    RigidBody.cpp
    World.cpp
    Integrator.cpp
    CollisionDetector.cpp
    CollisionResolver.cpp
)
target_include_directories(pysicd_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# --- Renderer: depende de SFML y del núcleo ---
add_library(pysicd_render STATIC Renderer.cpp)
target_link_libraries(pysicd_render PUBLIC pysicd_core SFML::Graphics)

# --- Ejecutable: solo si ya existe main.cpp ---

# --- Ejecutables de prueba ---
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/Main.cpp)
    add_executable(pysicd_caida Main.cpp)
    target_link_libraries(pysicd_caida PRIVATE pysicd_render)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/Main2.cpp)
    add_executable(pysicd_escenas Main2.cpp)
    target_link_libraries(pysicd_escenas PRIVATE pysicd_render)
endif()

# Avisos del compilador: ayudan a cazar errores
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    foreach(t pysicd_core pysicd_render)
        target_compile_options(${t} PRIVATE -Wall -Wextra)
    endforeach()
endif()