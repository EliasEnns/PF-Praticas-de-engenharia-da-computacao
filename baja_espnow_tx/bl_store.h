// ============================================================================
//  BajaLink — buffer store-and-forward do sensor
// ----------------------------------------------------------------------------
//  Usado somente pelo sketch TX.
//
//  Problema que este arquivo resolve: a pista não é um laboratório. O carro
//  entra atrás de um barranco, passa por trás da arquibancada, o piloto
//  encosta o chassi entre o módulo e o box, e o link some por alguns
//  segundos. Num datalogger ingênuo esses segundos simplesmente não existem
//  no arquivo final, e é justamente aí que costuma estar o evento que se
//  queria analisar.
//
//  A saída é desacoplar a aquisição da transmissão. O sensor amostra sempre,
//  guarda o que ainda não foi confirmado num buffer circular em RAM, e drena
//  o buffer quando o link volta — mais rápido do que a taxa de aquisição,
//  porque a janela de congestionamento abre além do regime permanente. O
//  resultado é que uma queda de link vira latência, não lacuna, desde que
//  caiba no buffer.
//
//  Dimensionamento: a profundidade é escolhida em runtime. Sem PSRAM são
//  BL_SLOTS_INTERNAL slots em SRAM; com PSRAM detectada, BL_SLOTS_PSRAM. A
//  256 B por slot e ~50 quadros/s, isso é cerca de 5 s contra 80 s de
//  autonomia de queda — e os devkits ESP32-S3-DevKitC-1 das variantes R2/R8
//  têm PSRAM, então vale detectar em vez de fixar.
//
//  O que o buffer NÃO faz: ele não é persistente. Um reset do sensor perde o
//  conteúdo. Era o papel do cartão SD previsto na proposta; sem a placa, o
//  buffer em RAM cobre a queda de link, não a queda de alimentação.
// ============================================================================
#ifndef BL_STORE_H
#define BL_STORE_H

// Condicional para que este header tambem compile no PC, onde o teste de
// integracao o exercita sem o core do Arduino.
#if __has_include(<Arduino.h>)
  #include <Arduino.h>        // psramFound()
#endif
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <esp_heap_caps.h>
#include "bl_config.h"
#include "bl_proto.h"

typedef struct {
  uint16_t seq;
  uint16_t len;
  uint8_t  flags;
  uint8_t  tries;       // transmissões já feitas deste quadro
  uint8_t  recs;        // records embutidos (para contabilizar descarte)
  uint8_t  used;
  uint32_t t_first_ms;  // primeira transmissão (latência fim-a-fim)
  uint32_t t_last_ms;   // última transmissão (base do RTO)
  uint8_t  data[BL_MAX_PAYLOAD];
} bl_slot_t;

typedef struct {
  bl_slot_t *slots;
  uint16_t   n;          // profundidade efetiva
  uint16_t   base_seq;   // mais antigo não confirmado
  uint16_t   next_seq;   // próximo seq a atribuir
  uint8_t    psram;      // 1 = alocado em PSRAM
  // Contadores vitalícios
  uint32_t   pushed;      // quadros enfileirados
  uint32_t   dropped;     // records perdidos por ESTOURO de buffer
  uint32_t   giveup;      // quadros abandonados por esgotar as tentativas
  uint32_t   giveup_recs; // records perdidos nesses quadros abandonados
  uint16_t   peak_used;  // ocupação máxima vista (dimensionamento)
} bl_store_t;

static bl_store_t bl_sf;

// ---------------------------------------------------------------------------
// Inicialização. Tenta PSRAM; cai para SRAM interna; em último caso reduz a
// profundidade até caber, para nunca travar o boot por falta de heap.
// ---------------------------------------------------------------------------
static inline bool bl_store_init(void) {
  memset(&bl_sf, 0, sizeof(bl_sf));

#if CONFIG_SPIRAM
  if (psramFound()) {
    bl_sf.slots = (bl_slot_t *)heap_caps_malloc(
        (size_t)BL_SLOTS_PSRAM * sizeof(bl_slot_t), MALLOC_CAP_SPIRAM);
    if (bl_sf.slots) {
      bl_sf.n = BL_SLOTS_PSRAM;
      bl_sf.psram = 1;
    }
  }
#endif

  if (!bl_sf.slots) {
    uint16_t want = BL_SLOTS_INTERNAL;
    while (want >= 32 && !bl_sf.slots) {
      bl_sf.slots = (bl_slot_t *)malloc((size_t)want * sizeof(bl_slot_t));
      if (bl_sf.slots) bl_sf.n = want;
      else want = (uint16_t)(want / 2);
    }
  }
  if (!bl_sf.slots) return false;

  for (uint16_t i = 0; i < bl_sf.n; i++) bl_sf.slots[i].used = 0;
  return true;
}

static inline uint16_t bl_store_used(void) {
  return (uint16_t)bl_seq_diff(bl_sf.next_seq, bl_sf.base_seq);
}

static inline uint16_t bl_store_free(void) {
  return (uint16_t)(bl_sf.n - bl_store_used());
}

// Slot de um seq, se ainda estiver na janela e vivo.
static inline bl_slot_t *bl_store_get(uint16_t seq) {
  if (!bl_seq_ge(seq, bl_sf.base_seq)) return NULL;
  if (!bl_seq_gt(bl_sf.next_seq, seq)) return NULL;
  bl_slot_t *s = &bl_sf.slots[seq % bl_sf.n];
  return s->used ? s : NULL;
}

// Libera o mais antigo. Usado pela política de estouro e pela desistência.
static inline void bl_store_drop_oldest(void) {
  if (bl_store_used() == 0) return;
  bl_slot_t *s = &bl_sf.slots[bl_sf.base_seq % bl_sf.n];
  if (s->used) {
    bl_sf.dropped += s->recs;
    s->used = 0;
  }
  bl_sf.base_seq++;
}

// ---------------------------------------------------------------------------
// Enfileira um quadro já montado (payload de DATA). Devolve o seq atribuído
// em *out_seq. Retorna false somente quando a política é descartar o novo e
// o buffer está cheio.
// ---------------------------------------------------------------------------
static inline bool bl_store_push(const uint8_t *payload, uint16_t len,
                                 uint8_t recs, uint8_t flags,
                                 uint16_t *out_seq) {
  if (len > BL_MAX_PAYLOAD) return false;

  if (bl_store_free() == 0) {
#if BL_OVERFLOW_DROP_OLDEST
    // Mantém o dado recente: o dashboard ao vivo é mais útil com os últimos
    // segundos do que com os primeiros de uma queda longa.
    bl_store_drop_oldest();
    flags |= BL_F_GAP;
#else
    bl_sf.dropped += recs;
    return false;
#endif
  }

  uint16_t seq = bl_sf.next_seq;
  bl_slot_t *s = &bl_sf.slots[seq % bl_sf.n];
  s->seq        = seq;
  s->len        = len;
  s->flags      = flags;
  s->tries      = 0;
  s->recs       = recs;
  s->used       = 1;
  s->t_first_ms = 0;
  s->t_last_ms  = 0;
  memcpy(s->data, payload, len);

  bl_sf.next_seq++;
  bl_sf.pushed++;
  uint16_t u = bl_store_used();
  if (u > bl_sf.peak_used) bl_sf.peak_used = u;
  if (out_seq) *out_seq = seq;
  return true;
}

// ---------------------------------------------------------------------------
// ACK cumulativo: libera tudo abaixo de ack_base. Devolve quantos quadros
// saíram da janela.
// ---------------------------------------------------------------------------
static inline uint16_t bl_store_ack_upto(uint16_t ack_base) {
  uint16_t freed = 0;
  while (bl_seq_gt(ack_base, bl_sf.base_seq) && bl_store_used() > 0) {
    bl_slot_t *s = &bl_sf.slots[bl_sf.base_seq % bl_sf.n];
    s->used = 0;
    bl_sf.base_seq++;
    freed++;
  }
  return freed;
}

// ---------------------------------------------------------------------------
// ACK seletivo: marca um seq isolado como entregue. Não move a base — isso é
// papel do ACK cumulativo —, apenas evita retransmitir o que já chegou.
// ---------------------------------------------------------------------------
static inline void bl_store_ack_one(uint16_t seq) {
  bl_slot_t *s = bl_store_get(seq);
  if (s) s->used = 0;
}

// Descarta a cabeça da janela quando ela esgotou as tentativas, para que um
// único quadro irrecuperável não bloqueie a drenagem do resto.
// As duas perdas possiveis tem causas distintas e nao devem ser somadas no
// mesmo contador: estouro de buffer significa que a queda de link durou mais
// que a autonomia do buffer, e pede buffer maior ou buffer em disco; abandono
// por prazo significa que aquele quadro especifico nao foi recuperado dentro
// do teto de latencia, e pede link melhor. Com os dois juntos num unico
// numero nao se sabe qual dos dois problemas se tem.
static inline void bl_store_giveup_head(void) {
  bl_slot_t *s = &bl_sf.slots[bl_sf.base_seq % bl_sf.n];
  if (s->used) {
    bl_sf.giveup++;
    bl_sf.giveup_recs += s->recs;
    s->used = 0;
  }
  bl_sf.base_seq++;
}

#endif // BL_STORE_H
