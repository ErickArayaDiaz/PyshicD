#pragma once
#include "World.h"

enum class Escena { Caja, Anillo, Cuenco, Rampas };

const char* nombreEscena(Escena e);
void cargarEscena(World& world, Escena e, float ancho, float alto);