// Stub mínimo para rodar os testes de lógica do BajaLink no host.
#pragma once
#include <stdlib.h>
#define MALLOC_CAP_SPIRAM 1
static inline void *heap_caps_malloc(size_t n, int caps) { (void)caps; return malloc(n); }
static inline bool psramFound(void) { return false; }
