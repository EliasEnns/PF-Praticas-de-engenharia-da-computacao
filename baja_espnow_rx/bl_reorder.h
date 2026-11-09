// ============================================================================
//  BajaLink — janela de reordenação do gateway
// ----------------------------------------------------------------------------
//  Usado somente pelo sketch RX.
//
//  Com retransmissão, os quadros deixam de chegar em ordem: o quadro 100 se
//  perde, 101 a 105 chegam, e 100 só aparece depois de retransmitido. Entregar
//  isso direto ao dashboard produziria um log com o tempo andando para trás,
//  o que estraga qualquer análise e atrapalha até o gráfico ao vivo.
//
//  A janela guarda o que chega fora de ordem e libera em sequência estrita.
//  O ponto delicado é quanto esperar por um quadro faltante: esperar para
//  sempre congela a tela, não esperar reintroduz a desordem. A escolha aqui é
//  um prazo (BL_RX_HOLE_TIMEOUT_MS) após o qual a lacuna é declarada,
//  registrada e pulada. A latência de pior caso fica limitada e explícita, e
//  a perda fica contabilizada em vez de silenciosa.
//
//  Há um atalho para o caso mais comum de lacuna definitiva: quando o buffer
//  do sensor estoura, os quadros descartados nunca serão retransmitidos.
//  O sensor informa a base da sua janela no quadro STAT, e o gateway encerra
//  a lacuna na hora, sem gastar o prazo inteiro esperando por um quadro que
//  não existe mais.
// ============================================================================
#ifndef BL_REORDER_H
#define BL_REORDER_H

#include <stdint.h>
#include <string.h>
#include "bl_config.h"
#include "bl_proto.h"

typedef struct {
  uint8_t  used;
  uint8_t  flags;
  uint16_t seq;          // redundante com o indice; guardado como verificacao
  uint16_t len;
  uint8_t  data[BL_MAX_PAYLOAD];
} bl_rslot_t;

typedef struct {
  bl_rslot_t slots[BL_RX_WIN];
  uint16_t   base;         // próximo seq a entregar
  uint8_t    synced;       // já viu o primeiro DATA desta associação
  uint32_t   t_wait;       // instante em que a base começou a esperar
  // Contadores vitalícios
  uint32_t   frames;       // quadros únicos entregues
  uint32_t   dups;         // duplicatas descartadas
  uint32_t   gaps;         // quadros declarados perdidos
  uint32_t   bad;          // quadros rejeitados por CRC/versão
  uint16_t   peak_depth;   // ocupação máxima da janela
} bl_rx_t;

static bl_rx_t bl_rx;

enum { BL_RX_OK = 0, BL_RX_DUP = 1, BL_RX_WINFULL = 2 };

static inline void bl_rx_init(void) {
  memset(&bl_rx, 0, sizeof(bl_rx));
}

// Chamado ao (re)associar: a próxima sequência vista define a base, o que
// dispensa qualquer acordo prévio de numeração entre os dois lados e faz o
// gateway sobreviver a um reset do sensor.
static inline void bl_rx_unsync(void) {
  for (uint16_t i = 0; i < BL_RX_WIN; i++) bl_rx.slots[i].used = 0;
  bl_rx.synced = 0;
}

static inline uint16_t bl_rx_depth(void) {
  uint16_t d = 0;
  for (uint16_t i = 0; i < BL_RX_WIN; i++) if (bl_rx.slots[i].used) d++;
  return d;
}

// ---------------------------------------------------------------------------
// Armazena um quadro DATA.
// ---------------------------------------------------------------------------
static inline uint8_t bl_rx_store(uint16_t seq, uint8_t flags,
                                  const uint8_t *data, uint16_t len,
                                  uint32_t now) {
  if (len > BL_MAX_PAYLOAD) return BL_RX_DUP;

  if (!bl_rx.synced) {
    bl_rx.base   = seq;
    bl_rx.synced = 1;
    bl_rx.t_wait = now;
  }

  int32_t d = bl_seq_diff(seq, bl_rx.base);

  if (d < 0) {                       // já entregue: retransmissão redundante
    bl_rx.dups++;
    return BL_RX_DUP;
  }

  if (d >= (int32_t)BL_RX_WIN) {
    // O sensor esta a frente do que a janela alcanca. Acontece depois de uma
    // queda longa, em que ele descartou quadros que nunca serao retransmitidos.
    //
    // A primeira versao deste trecho limpava a janela inteira e reancorava na
    // sequencia recebida. Estava errado: jogava fora quadros corretamente
    // recebidos que ainda seriam entregues, e com isso transformava uma
    // lacuna pequena numa perda grande. O teste de integracao pegou isso.
    //
    // A versao correta so avanca sobre lacunas. Se a cabeca da janela ja
    // chegou, a entrega em ordem vai drenar a janela na mesma passagem do
    // laco, e este quadro sera retransmitido. Nenhum quadro recebido e
    // descartado para abrir espaco.
    while (bl_seq_diff(seq, bl_rx.base) >= (int32_t)BL_RX_WIN) {
      bl_rslot_t *h = &bl_rx.slots[bl_rx.base % BL_RX_WIN];
      if (h->used) break;
      bl_rx.gaps++;
      bl_rx.base++;
    }
    bl_rx.t_wait = now;
    d = bl_seq_diff(seq, bl_rx.base);
    if (d >= (int32_t)BL_RX_WIN) return BL_RX_WINFULL;
  }

  bl_rslot_t *s = &bl_rx.slots[seq % BL_RX_WIN];
  if (s->used) {                     // mesma posição, mesmo seq: duplicata
    bl_rx.dups++;
    return BL_RX_DUP;
  }
  s->used  = 1;
  s->flags = flags;
  s->seq   = seq;
  s->len   = len;
  memcpy(s->data, data, len);

  uint16_t depth = bl_rx_depth();
  if (depth > bl_rx.peak_depth) bl_rx.peak_depth = depth;
  return BL_RX_OK;
}

// Slot pronto na cabeça da janela, ou NULL.
static inline bl_rslot_t *bl_rx_head(void) {
  if (!bl_rx.synced) return NULL;
  bl_rslot_t *s = &bl_rx.slots[bl_rx.base % BL_RX_WIN];
  if (!s->used) return NULL;
  if (s->seq != bl_rx.base) {   // nao deve acontecer; se acontecer, nao entrega
    s->used = 0;
    bl_rx.bad++;
    return NULL;
  }
  return s;
}

static inline void bl_rx_pop(uint32_t now) {
  bl_rslot_t *s = &bl_rx.slots[bl_rx.base % BL_RX_WIN];
  s->used = 0;
  bl_rx.base++;
  bl_rx.frames++;
  bl_rx.t_wait = now;
}

// Existe algo adiante esperando a lacuna ser resolvida?
static inline bool bl_rx_has_ahead(void) {
  for (uint16_t i = 1; i < BL_RX_WIN; i++)
    if (bl_rx.slots[(uint16_t)(bl_rx.base + i) % BL_RX_WIN].used) return true;
  return false;
}

// Declara a cabeça perdida e segue adiante.
static inline void bl_rx_skip(uint32_t now) {
  bl_rx.gaps++;
  bl_rx.base++;
  bl_rx.t_wait = now;
}

// Encerramento antecipado de lacuna a partir da base informada pelo sensor.
// Devolve quantos quadros foram declarados perdidos.
static inline uint16_t bl_rx_forward_to(uint16_t tx_base, uint32_t now) {
  uint16_t skipped = 0;
  if (!bl_rx.synced) return 0;
  while (bl_seq_gt(tx_base, bl_rx.base) && skipped < BL_RX_WIN) {
    bl_rslot_t *s = &bl_rx.slots[bl_rx.base % BL_RX_WIN];
    if (s->used) break;              // chegou: entrega normalmente
    bl_rx.gaps++;
    bl_rx.base++;
    skipped++;
  }
  if (skipped) bl_rx.t_wait = now;
  return skipped;
}

// ---------------------------------------------------------------------------
// Monta o ACK: base cumulativa + mapa seletivo dos 32 seq seguintes.
// ---------------------------------------------------------------------------
static inline void bl_rx_build_ack(bl_ack_t *a, uint16_t echo_seq, int8_t rssi) {
  memset(a, 0, sizeof(*a));
  a->ack_base = bl_rx.base;
  a->echo_seq = echo_seq;
  a->rssi     = rssi;
  uint32_t bits = 0;
  for (uint8_t i = 0; i < 32; i++) {
    uint16_t seq = (uint16_t)(bl_rx.base + 1 + i);
    if (bl_rx.slots[seq % BL_RX_WIN].used) bits |= (1u << i);
  }
  a->sack_bits = bits;
  a->rx_frames = bl_rx.frames;
  a->rx_dups   = bl_rx.dups;
  a->rx_gaps   = bl_rx.gaps;
  uint16_t d   = bl_rx_depth();
  a->rx_free   = (uint8_t)((BL_RX_WIN - d) > 255 ? 255 : (BL_RX_WIN - d));
}

#endif // BL_REORDER_H
