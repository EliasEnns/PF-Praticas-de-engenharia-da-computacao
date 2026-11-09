#ifndef BL_JSON_H
#define BL_JSON_H
// ============================================================================
//  BajaLink — montagem da linha de saída do gateway
// ----------------------------------------------------------------------------
//  Usado somente pelo sketch RX.
//
//  Está num arquivo separado, sem depender de Serial nem de nada do Arduino,
//  por um motivo prático: é o único ponto do sistema em que o firmware e o
//  dashboard têm de concordar byte a byte. Isolado assim, a mesma função pode
//  ser compilada no PC e testada contra o parser Python de verdade, o que
//  transforma "deve funcionar" em "foi verificado". O teste em
//  tools/test_bajalink.cpp gera linhas com esta função e
//  tools/test_formato.py as passa pelo parser do dashboard.
//
//  Contrato de saída: uma linha, um objeto JSON, o record original intacto
//  mais dois objetos anexados.
//
//    "tele"  — apenas se o record não trouxe o seu. Preserva os campos que o
//              dashboard já exibia no caminho LoRa (rx, lost, loss_pct, rst),
//              agora alimentados pelos contadores do ESP-NOW.
//    "link"  — sempre. Saúde completa do link. As chaves são curtas porque
//              isto vai em todo record: a ~50 records/s, cada byte aqui custa
//              50 B/s na porta serial. A legenda está em
//              docs/BAJALINK_PROTOCOLO.md e em LINK_FIELDS, no dashboard.
// ============================================================================

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "bl_config.h"

typedef struct {
  // Contadores do gateway
  uint32_t rx_frames;
  uint32_t rx_dups;
  uint32_t rx_gaps;
  uint32_t link_drops;
  uint16_t reorder;
  int8_t   rssi;
  float    pdr;
  uint8_t  up;
  uint8_t  channel;
  // Espelho do último STAT do sensor
  uint32_t tx_retx;
  uint32_t tx_dropped;
  uint32_t tx_giveup;
  uint16_t buf_used;
  uint16_t buf_size;
  uint16_t cwnd;
  uint16_t srtt;
  uint16_t rto;
  uint16_t src_hz;
  uint8_t  lr;
  uint8_t  src;
} bl_json_ctx_t;

// Busca ingênua de subcadeia num buffer sem terminador. O record tem ~220 B e
// a agulha 6 B; qualquer coisa mais esperta seria otimização prematura.
static inline bool bl_json_contains(const uint8_t *hay, uint16_t n,
                                    const char *needle) {
  uint16_t m = (uint16_t)strlen(needle);
  if (m == 0 || n < m) return false;
  for (uint16_t i = 0; (uint16_t)(i + m) <= n; i++)
    if (memcmp(hay + i, needle, m) == 0) return true;
  return false;
}

// Tamanho mínimo do buffer de saída.
#define BL_JSON_OUT_CAP  (BL_MAX_RECORD + 416)

// Escreve a linha decorada em out (sem '\n'). Devolve o número de bytes, ou
// -1 se o record não parece um objeto JSON — nesse caso o chamador deve
// repassar o record cru, porque inventar formato é pior que entregar o que
// veio e deixar o dashboard descartar.
static inline int bl_json_decorate(char *out, size_t cap,
                                   const uint8_t *rec, uint16_t len,
                                   const bl_json_ctx_t *c) {
  if (len < 2 || rec[0] != '{' || rec[len - 1] != '}') return -1;
  if ((size_t)len + 416 > cap) return -1;

  int n = (int)(len - 1);              // copia o record sem a chave final
  memcpy(out, rec, (size_t)n);

  if (!bl_json_contains(rec, len, "\"tele\"")) {
    uint32_t tot = c->rx_frames + c->rx_gaps;
    float lp = tot ? (100.0f * (float)c->rx_gaps / (float)tot) : 0.0f;
    n += snprintf(out + n, cap - (size_t)n,
                  ",\"tele\":{\"rx\":%lu,\"lost\":%lu,\"loss_pct\":%.2f,\"rst\":%lu}",
                  (unsigned long)c->rx_frames, (unsigned long)c->rx_gaps,
                  (double)lp, (unsigned long)c->link_drops);
  }

  n += snprintf(out + n, cap - (size_t)n,
      ",\"link\":{\"rssi\":%d,\"pdr\":%.2f,\"fr\":%lu,\"dup\":%lu,\"gap\":%lu,"
      "\"ro\":%u,\"retx\":%lu,\"drp\":%lu,\"gu\":%lu,\"buf\":%u,\"bmax\":%u,"
      "\"cwnd\":%u,\"srtt\":%u,\"rto\":%u,\"ch\":%u,\"lr\":%u,\"src\":%u,"
      "\"hz\":%u,\"up\":%u,\"ld\":%lu}}",
      (int)c->rssi, (double)c->pdr,
      (unsigned long)c->rx_frames, (unsigned long)c->rx_dups,
      (unsigned long)c->rx_gaps, (unsigned)c->reorder,
      (unsigned long)c->tx_retx, (unsigned long)c->tx_dropped,
      (unsigned long)c->tx_giveup,
      (unsigned)c->buf_used, (unsigned)c->buf_size, (unsigned)c->cwnd,
      (unsigned)c->srtt, (unsigned)c->rto,
      (unsigned)c->channel, (unsigned)c->lr, (unsigned)c->src,
      (unsigned)c->src_hz, (unsigned)c->up, (unsigned long)c->link_drops);

  if (n > (int)cap - 2) n = (int)cap - 2;
  return n;
}

#endif // BL_JSON_H
